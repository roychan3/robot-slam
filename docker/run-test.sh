#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 3 ]]; then
  echo "Usage: $0 ROS_TEST_PROGRAM [STARTUP_TIMEOUT_SECONDS] [TEST_TIMEOUT_SECONDS]" >&2
  exit 2
fi

test_program="$1"
startup_timeout="${2:-30}"
test_timeout="${3:-120}"

if [[ ! "${test_program}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "Invalid ROS test program: ${test_program}" >&2
  exit 2
fi
if [[ ! "${startup_timeout}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Startup timeout must be a positive integer" >&2
  exit 2
fi
if [[ ! "${test_timeout}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Test timeout must be a positive integer" >&2
  exit 2
fi

image="${ROBOT_SLAM_IMAGE:-robot-slam:noetic}"
container="robot-slam-test-${test_program%.py}-$$"

cleanup() {
  docker stop "${container}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# --shm-size matches docker-compose.yml. Gazebo's rendering path outgrows the
# 64 MB default, and without this the tests would exercise a different
# configuration from `docker compose up`.
docker run --rm --detach --shm-size=1g --name "${container}" "${image}" >/dev/null

ros_ready=false
for ((attempt = 1; attempt <= startup_timeout; attempt++)); do
  if docker exec "${container}" bash -c \
    'source /opt/ros/noetic/setup.bash && rosnode list >/dev/null 2>&1'; then
    ros_ready=true
    break
  fi
  sleep 1
done

if [[ "${ros_ready}" != true ]]; then
  echo "ROS master did not become ready within ${startup_timeout} seconds" >&2
  docker logs "${container}" >&2
  exit 1
fi

set +e
docker exec "${container}" \
  timeout --foreground --signal=TERM --kill-after=10s "${test_timeout}s" \
  bash -c \
  'source /opt/ros/noetic/setup.bash
   source /app/catkin_ws/devel/setup.bash
   exec rosrun robot_slam "$1"' \
  _ "${test_program}"
test_status=$?
set -e

if [[ ${test_status} -eq 124 || ${test_status} -eq 137 ]]; then
  echo "ROS test ${test_program} exceeded ${test_timeout} seconds" >&2
fi
exit "${test_status}"
