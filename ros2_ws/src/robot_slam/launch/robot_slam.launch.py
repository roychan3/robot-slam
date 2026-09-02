"""Launch the complete Gazebo Harmonic indoor mapping demonstration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("robot_slam")
    launch_dir = PathJoinSubstitution([package_share, "launch"])

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    paused = LaunchConfiguration("paused")
    start_octomap = LaunchConfiguration("start_octomap")
    start_slam_toolbox = LaunchConfiguration("start_slam_toolbox")
    start_rbpf_slam = LaunchConfiguration("start_rbpf_slam")
    rbpf_publish_tf = LaunchConfiguration("rbpf_publish_tf")
    rbpf_map_frame = LaunchConfiguration("rbpf_map_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("start_octomap", default_value="true"),
            DeclareLaunchArgument("start_slam_toolbox", default_value="true"),
            DeclareLaunchArgument("start_rbpf_slam", default_value="false"),
            DeclareLaunchArgument("rbpf_publish_tf", default_value="false"),
            DeclareLaunchArgument("rbpf_map_frame", default_value="rbpf_map"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "simulation.launch.py"])
                ),
                launch_arguments={"gui": gui, "paused": paused}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "slam.launch.py"])
                ),
                launch_arguments={
                    "start_octomap": start_octomap,
                    "start_slam_toolbox": start_slam_toolbox,
                    "start_rbpf_slam": start_rbpf_slam,
                    "rbpf_publish_tf": rbpf_publish_tf,
                    "rbpf_map_frame": rbpf_map_frame,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(rviz),
                arguments=[
                    "-d",
                    PathJoinSubstitution([package_share, "config", "slam.rviz"]),
                ],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
