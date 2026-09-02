"""Launch selectable 2D SLAM backends and optional 3D OctoMap mapping."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("robot_slam")
    slam_config = PathJoinSubstitution(
        [package_share, "config", "slam_toolbox.yaml"]
    )
    rbpf_config = PathJoinSubstitution(
        [package_share, "config", "rbpf_slam.yaml"]
    )
    slam_toolbox_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )

    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    start_octomap = LaunchConfiguration("start_octomap")
    start_slam_toolbox = LaunchConfiguration("start_slam_toolbox")
    start_rbpf_slam = LaunchConfiguration("start_rbpf_slam")
    rbpf_publish_tf = LaunchConfiguration("rbpf_publish_tf")
    rbpf_map_frame = LaunchConfiguration("rbpf_map_frame")
    show_accuracy = LaunchConfiguration("show_accuracy")
    safe_rbpf_publish_tf = PythonExpression(
        [
            "'",
            rbpf_publish_tf,
            "'.lower() == 'true' and '",
            start_slam_toolbox,
            "'.lower() != 'true'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pointcloud_topic", default_value="/camera/depth/points"
            ),
            DeclareLaunchArgument("start_octomap", default_value="true"),
            DeclareLaunchArgument("start_slam_toolbox", default_value="true"),
            DeclareLaunchArgument("start_rbpf_slam", default_value="false"),
            DeclareLaunchArgument("rbpf_publish_tf", default_value="false"),
            DeclareLaunchArgument("rbpf_map_frame", default_value="rbpf_map"),
            DeclareLaunchArgument("show_accuracy", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                condition=IfCondition(start_slam_toolbox),
                launch_arguments={
                    "slam_params_file": slam_config,
                    "use_sim_time": "true",
                    "autostart": "true",
                }.items(),
            ),
            Node(
                package="robot_slam",
                executable="rbpf_slam_node",
                name="rbpf_slam",
                output="screen",
                condition=IfCondition(start_rbpf_slam),
                parameters=[
                    rbpf_config,
                    {
                        "use_sim_time": True,
                        "publish_tf": ParameterValue(
                            safe_rbpf_publish_tf, value_type=bool
                        ),
                        "map_frame": rbpf_map_frame,
                    },
                ],
            ),
            Node(
                package="robot_slam",
                executable="accuracy_monitor.py",
                name="accuracy_monitor",
                output="screen",
                condition=IfCondition(show_accuracy),
                parameters=[
                    {
                        "use_sim_time": True,
                        "map_frame": "map",
                        "base_frame": "base_footprint",
                        "ground_truth_topic": "/ground_truth/pose",
                        "rbpf_pose_topic": "/rbpf/pose",
                    }
                ],
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
