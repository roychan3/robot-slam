#!/usr/bin/env python3
"""Integration smoke test for the running Gazebo / ROS 2 SLAM graph."""

import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import LaserScan, PointCloud2


def wait(node, topic, msg_type, timeout=60.0, sensor_data=False):
    node.get_logger().info(f"Waiting for {topic}")
    qos = QoSProfile(depth=10)
    if sensor_data:
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
    if topic == "/map":
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    received, message = wait_for_message(
        msg_type, node, topic, qos_profile=qos, time_to_wait=timeout
    )
    if not received:
        raise TimeoutError(f"no {topic} message within {timeout:.0f} seconds")
    return message


def main():
    rclpy.init()
    node = rclpy.create_node("robot_slam_smoke_test")
    try:
        odom = wait(node, "/odom", Odometry)
        ground_truth = wait(node, "/ground_truth/pose", PoseStamped)
        scan = wait(node, "/scan", LaserScan, sensor_data=True)
        raw_cloud = wait(
            node, "/camera/depth/points", PointCloud2, sensor_data=True
        )
        grid = wait(node, "/map", OccupancyGrid)
        octomap_cloud = wait(
            node, "/octomap_point_cloud_centers", PointCloud2
        )

        finite_scan = sum(1 for value in scan.ranges if value < float("inf"))
        checks = {
            "odometry frame": odom.header.frame_id == "odom",
            "Gazebo ground truth": (
                ground_truth.header.frame_id == "robot_slam_indoor"
                and abs(ground_truth.pose.position.x) > 0.1
            ),
            "lidar returns": finite_scan > 20,
            "raw depth cloud": raw_cloud.width * raw_cloud.height > 100,
            "SLAM Toolbox grid": grid.info.width * grid.info.height > 0,
            "accumulated 3D cloud": (
                octomap_cloud.width * octomap_cloud.height > 0
            ),
        }
        failed = [name for name, passed in checks.items() if not passed]
        for name, passed in checks.items():
            node.get_logger().info(f"{name}: {'PASS' if passed else 'FAIL'}")
        if failed:
            node.get_logger().error(f"Failed checks: {', '.join(failed)}")
            return 1
        node.get_logger().info(
            "Gazebo Harmonic, TurtleBot3, SLAM Toolbox, and 3D point cloud "
            "are healthy"
        )
        return 0
    except TimeoutError as exc:
        node.get_logger().error(f"Smoke test timed out: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
