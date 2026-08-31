#!/usr/bin/env bash
set -euo pipefail

image="${ROBOT_SLAM_IMAGE:-robot-slam:noetic}"
container="robot-slam-smoke-$$"

cleanup() {
  docker stop "${container}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

docker run --rm --detach --name "${container}" "${image}" >/dev/null

ros_ready=false
for _ in $(seq 1 30); do
  if docker exec "${container}" bash -c \
    'source /opt/ros/noetic/setup.bash && rosnode list >/dev/null 2>&1'; then
    ros_ready=true
    break
  fi
  sleep 1
done

if [[ "${ros_ready}" != true ]]; then
  echo "ROS master did not become ready within 30 seconds" >&2
  docker logs "${container}" >&2
  exit 1
fi

docker exec "${container}" bash -c \
  'source /opt/ros/noetic/setup.bash && source /app/catkin_ws/devel/setup.bash && rosrun robot_slam smoke_test.py'
