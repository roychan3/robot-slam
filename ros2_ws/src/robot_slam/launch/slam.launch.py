"""Launch 2D SLAM Toolbox mapping and optional 3D OctoMap mapping."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("robot_slam")
    slam_config = PathJoinSubstitution(
        [package_share, "config", "slam_toolbox.yaml"]
    )
    slam_toolbox_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )

    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    start_octomap = LaunchConfiguration("start_octomap")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pointcloud_topic", default_value="/camera/depth/points"
            ),
            DeclareLaunchArgument("start_octomap", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                launch_arguments={
                    "slam_params_file": slam_config,
                    "use_sim_time": "true",
                    "autostart": "true",
                }.items(),
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                condition=IfCondition(start_octomap),
                remappings=[("cloud_in", pointcloud_topic)],
                parameters=[
                    {
                        "use_sim_time": True,
                        "frame_id": "map",
                        "base_frame_id": "base_footprint",
                        "resolution": 0.08,
                        "sensor_model.max_range": 6.0,
                        "sensor_model.hit": 0.70,
                        "sensor_model.miss": 0.40,
                        "filter_ground": False,
                        "latch": True,
                    }
                ],
            ),
        ]
    )
