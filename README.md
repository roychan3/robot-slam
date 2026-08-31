# TurtleBot3 indoor SLAM

This project runs a complete ROS 1 Noetic simulation in Docker:

- Gazebo 11 provides a 10 m × 8 m indoor world with rooms and furniture.
- One TurtleBot3 Waffle Pi–class robot provides differential-drive odometry,
  a 360° lidar, and an RGB-D camera.
- `slam_gmapping` builds the 2D occupancy map and publishes the `map` frame.
- `octomap_server` fuses RGB-D observations in that frame and publishes the
  indoor environment as a 3D `sensor_msgs/PointCloud2`.

`gmapping` is a 2D lidar SLAM algorithm; the RGB-D/OctoMap branch is what adds
the requested 3D representation.

## Run

Docker and Docker Compose are the only host requirements.

```bash
mkdir -p data
docker compose build
docker compose up -d
```

The default is headless: the container always creates its own virtual X
display, because Gazebo Classic needs a rendering context for its depth camera
even with `gzclient` disabled.

On Linux, `./data` is a bind mount that keeps host ownership, so the container
user must match the user that owns it. Start it with:

```bash
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up -d
```

macOS Docker Desktop maps ownership itself and needs no override. If you skip
this on Linux, the simulation still runs and the entrypoint prints a warning —
only saving to `/data` fails.

Drive the robot from another terminal so both maps cover the environment:

```bash
docker compose exec slam bash -c \
  'source /opt/ros/noetic/setup.bash && source /app/catkin_ws/devel/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py'
```

Save the accumulated 3D environment to the host's `data/` directory:

```bash
docker compose exec slam bash -c \
  'source /opt/ros/noetic/setup.bash && source /app/catkin_ws/devel/setup.bash && rosrun robot_slam save_pointcloud.py --output /data/indoor_environment.pcd'
```

Stop the simulation with:

```bash
docker compose down
```

## Main ROS interfaces

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 360° lidar input to gmapping |
| `/map` | `nav_msgs/OccupancyGrid` | 2D gmapping result |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Current RGB-D view |
| `/octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | Accumulated 3D environment |
| `/cmd_vel` | `geometry_msgs/Twist` | TurtleBot3 drive commands |
| `/odom` | `nav_msgs/Odometry` | Simulated wheel odometry |

## Verification

The integration check starts a temporary container and verifies live odometry,
lidar, depth cloud, gmapping map, and accumulated 3D cloud:

```bash
./docker/smoke-test.sh
```

To check an already-running Compose service instead:

```bash
docker compose exec slam bash -c \
  'source /opt/ros/noetic/setup.bash && source /app/catkin_ws/devel/setup.bash && rosrun robot_slam smoke_test.py'
```

## Gazebo GUI

The portable option is the browser-based UI, which also runs fully in Docker:

```bash
docker compose --profile ui up -d ui
```

Open <http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale>.

Ubuntu 20.04 ships noVNC 1.0, where `vnc.html` is the current entry point;
`vnc_auto.html` still works but is a compatibility shim from the 0.x series.
The port is published on loopback only, because x11vnc runs without a password
inside the container.

Stop it with:

```bash
docker compose --profile ui down
```
