"""Launch Gazebo Harmonic, the robot, and the Gazebo / ROS 2 bridge."""

from launch import LaunchDescription  # pyright: ignore[reportAttributeAccessIssue]
from launch.actions import (  # pyright: ignore[reportMissingImports]
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import (  # pyright: ignore[reportMissingImports]
    IfCondition,
    UnlessCondition,
)
from launch.launch_description_sources import (  # pyright: ignore[reportMissingImports]
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (  # pyright: ignore[reportMissingImports]
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node  # pyright: ignore[reportMissingImports]
from launch_ros.descriptions import (  # pyright: ignore[reportMissingImports]
    ParameterValue,
)
from launch_ros.substitutions import (  # pyright: ignore[reportMissingImports]
    FindPackageShare,
)


def generate_launch_description():
    package_share = FindPackageShare("robot_slam")
    world = PathJoinSubstitution([package_share, "worlds", "indoor.world"])
    model = PathJoinSubstitution(
        [package_share, "urdf", "turtlebot3_waffle_pi_rgbd.urdf.xacro"]
    )
    bridge_config = PathJoinSubstitution(
        [package_share, "config", "ros_gz_bridge.yaml"]
    )
    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    run = PythonExpression(["'' if '", paused, "'.lower() == 'true' else '-r '"])

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("x", default_value="-3.0"),
            DeclareLaunchArgument("y", default_value="-1.8"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    "gz_args": [run, "-v 2 ", world],
                    "on_exit_shutdown": "true",
                }.items(),
                condition=IfCondition(gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    "gz_args": [run, "-s -v 2 ", world],
                    "on_exit_shutdown": "true",
                }.items(),
                condition=UnlessCondition(gui),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "publish_frequency": 30.0,
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_turtlebot3",
                output="screen",
                arguments=[
                    "-name",
                    "turtlebot3_waffle_pi",
                    "-topic",
                    "robot_description",
                    "-x",
                    x,
                    "-y",
                    y,
                    "-z",
                    "0.02",
                    "-Y",
                    yaw,
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                parameters=[{"config_file": bridge_config}],
            ),
        ]
    )
