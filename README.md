# TurtleBot3 indoor SLAM

This project runs a complete ROS 2 Jazzy simulation in Docker:

- Gazebo Harmonic provides a 10 m × 8 m indoor world with rooms and furniture.
- One TurtleBot3 Waffle Pi–class robot provides differential-drive odometry,
  a 360° lidar, and an RGB-D camera.
- SLAM Toolbox builds the 2D occupancy map and publishes the `map` frame.
- `octomap_server` fuses RGB-D observations in that frame and publishes the
  indoor environment as a 3D `sensor_msgs/PointCloud2`.

SLAM Toolbox handles 2D lidar SLAM; the RGB-D/OctoMap branch adds the 3D
representation. Gazebo and ROS communicate through `ros_gz_bridge` while the
public ROS topic names remain unchanged from the earlier Classic-based stack.

An optional educational Rao-Blackwellized particle-filter (RBPF) backend is
also included. It can run beside SLAM Toolbox on the same lidar and odometry
stream, publishing separate outputs so the two estimates can be compared.

## Run

Docker and Docker Compose are the only host requirements.

```bash
mkdir -p data
docker compose build
docker compose up -d
```

The default is headless: the container always creates its own virtual X
display, because Gazebo Sim needs a rendering context for its depth camera
even with the graphical client disabled.

On Linux, `./data` is a bind mount that keeps host ownership, so the container
user must match the user that owns it. Start it with:

```bash
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up -d
```

macOS Docker Desktop maps ownership itself and needs no override. If you skip
this on Linux, the simulation still runs and the entrypoint prints a warning —
only saving to `/data` fails.

Drive the robot manually from another terminal so both maps cover the
environment:

```bash
docker compose exec slam bash -c \
  'source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard'
```

### Run both 2D SLAM algorithms

SLAM Toolbox remains the default. The `ui` profile already runs both, with the
two maps overlaid in RViz:

```bash
docker compose --profile ui up -d ui
```

Headless, without the browser view:

```bash
docker compose run --rm slam ros2 launch robot_slam robot_slam.launch.py \
  gui:=false start_rbpf_slam:=true rbpf_map_frame:=map
```

Both estimators consume `/scan` and `/odom`. SLAM Toolbox continues to publish
`/map` and the `map -> odom` transform; RBPF publishes `/rbpf/map` and
`/rbpf/pose`. RBPF deliberately does not publish TF in this mode, avoiding two
SLAM nodes assigning competing parents to `odom` — the launch derives that from
`start_slam_toolbox` rather than exposing it, so the conflict is unreachable.

Both commands pass `rbpf_map_frame:=map` so the two grids share a frame and can
be compared directly. RBPF's own default is a separate `rbpf_map` frame, which
keeps the topics independent but leaves nothing publishing a transform to that
frame, so RViz cannot draw it.

To run RBPF by itself as the primary 2D backend:

```bash
docker compose run --rm slam ros2 launch robot_slam robot_slam.launch.py \
  gui:=false start_slam_toolbox:=false start_rbpf_slam:=true \
  rbpf_publish_tf:=true rbpf_map_frame:=map
```

RBPF tuning parameters—including particle count, motion noise, scan-matching
window, and map resolution—are in `config/rbpf_slam.yaml`. The implementation
uses a deterministic random seed by default so repeated comparison runs are
reproducible.

Save the accumulated 3D environment to the host's `data/` directory:

```bash
docker compose exec slam bash -c \
  'source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 run robot_slam save_pointcloud.py --output /data/indoor_environment.pcd'
```

Stop the simulation with:

```bash
docker compose down
```

## Automatic loop-closure simulation

The loop-closure check drives the robot around a 2.0 m x 1.5 m rectangular
circuit in the open starting room, returns it to its original position and
heading, and then compares both simulated odometry and SLAM Toolbox's pose
with their starting values. The robot stops and the command exits non-zero if
it encounters an obstacle, times out, skips the circuit, or exceeds a closure
tolerance.

Build the image, then run the self-contained check:

```bash
docker compose build
./docker/loop-test.sh
```

To run the same loop in an already-running Compose service:

```bash
docker compose exec slam bash -c \
  'source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 run robot_slam loop_closure_test.py'
```

A successful run ends with `LOOP_CLOSURE_RESULT: PASS` and reports the driven
path length plus position/heading errors for odometry and SLAM. The defaults
require the final SLAM pose to be within 0.30 m and 0.25 rad of its initial
pose. Parameters such as `loop_width`, `loop_height`, and
`slam_position_tolerance` can be overridden with ROS 2 parameter syntax, for
example `--ros-args -p loop_width:=1.5`.

Exit codes distinguish the two ways a run can end badly:

| Code | Meaning |
| --- | --- |
| `0` | every check passed |
| `1` | the robot drove the circuit but a tolerance was exceeded |
| `2` | the run could not be completed — obstacle ahead, motion timeout, stale lidar, or a missing topic or transform |

`motion_timeout` is measured in **simulated** time, so it does not shrink when
Gazebo runs below real time. `./docker/run-test.sh` additionally caps each test
in wall-clock seconds, so a wedged simulation cannot hang the run.

## Main ROS interfaces

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | 360° lidar input to both 2D SLAM backends |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 2D SLAM Toolbox result |
| `/rbpf/map` | `nav_msgs/msg/OccupancyGrid` | Optional RBPF occupancy map |
| `/rbpf/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Optional RBPF pose estimate |
| `/rbpf/particles` | `geometry_msgs/msg/PoseArray` | Current RBPF particle poses shown as orange arrows in RViz |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | Current RGB-D view |
| `/octomap_point_cloud_centers` | `sensor_msgs/msg/PointCloud2` | Accumulated 3D environment |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | TurtleBot3 drive commands |
| `/odom` | `nav_msgs/msg/Odometry` | Simulated wheel odometry |
| `/ground_truth/pose` | `geometry_msgs/msg/PoseStamped` | Gazebo physics pose used only for live accuracy measurement |

## Verification

### Static analysis

Run BasedPyright from the repository root:

```bash
basedpyright
```

The checked Python code must have parameter type annotations. ROS 2 packages
are installed only in the Docker image, so their known imports have narrowly
scoped Pyright exceptions; new missing imports and other type errors are still
reported. A successful check ends with:

```text
0 errors, 0 warnings, 0 notes
```

To check Python syntax without installing BasedPyright, run:

```bash
python3 -m compileall -q ros2_ws/src/robot_slam
```

### Integration test

The integration check starts a temporary container and verifies live odometry,
Gazebo ground truth, lidar, depth cloud, SLAM Toolbox map, and accumulated 3D
cloud:

```bash
./docker/smoke-test.sh
```

To check an already-running Compose service instead:

```bash
docker compose exec slam bash -c \
  'source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 run robot_slam smoke_test.py'
```

## Browser UI

The browser-based UI runs fully in Docker and includes both the Gazebo camera
view and an RViz view of the map reconstructed by the robot:

```bash
docker compose --profile ui up -d ui
```

Open
<http://127.0.0.1:6080/jazzy-harmonic/vnc.html?autoconnect=true&resize=scale>.

RViz opens on the live `/map` occupancy grid and overlays the robot, current
lidar returns, and accumulated 3D reconstruction. The UI places Gazebo and
RViz side by side, so the simulation and the map stay visible together. The
map fills in as the robot is driven around the environment. SLAM Toolbox uses
the grayscale map layer, while RBPF uses the costmap color palette. Orange
arrows show the actual RBPF particle population; red points are the live lidar
transformed with SLAM Toolbox's pose. An always-visible
accuracy panel compares SLAM Toolbox and RBPF localization against Gazebo's
independent physics pose. For each algorithm it shows the current position and
heading error followed by the running root-mean-square error (RMSE); lower
values are better. Each estimator's initial frame is aligned to ground truth,
so the values measure drift and scan-matching corrections rather than the
arbitrary choice of map origin.

The release-specific URL prevents browsers from mixing cached noVNC 1.0 files
from the former Ubuntu 20.04 image with the noVNC 1.3 client in Ubuntu 24.04.
The port is published on loopback only, because x11vnc runs without a password
inside the container.

Stop it with:

```bash
docker compose --profile ui down
```
