#!/usr/bin/env python3
"""Integration smoke test for the running Gazebo/SLAM graph."""

import sys

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2


def wait(topic, msg_type, timeout=60.0):
    rospy.loginfo("Waiting for %s", topic)
    return rospy.wait_for_message(topic, msg_type, timeout=timeout)


def main():
    rospy.init_node("robot_slam_smoke_test", anonymous=True)
    try:
        odom = wait("/odom", Odometry)
        scan = wait("/scan", LaserScan)
        raw_cloud = wait("/camera/depth/points", PointCloud2)
        grid = wait("/map", OccupancyGrid)
        octomap_cloud = wait("/octomap_point_cloud_centers", PointCloud2)
    except rospy.ROSException as exc:
        rospy.logerr("Smoke test timed out: %s", exc)
        return 1

    finite_scan = sum(1 for value in scan.ranges if value < float("inf"))
    checks = {
        "odometry frame": odom.header.frame_id == "odom",
        "lidar returns": finite_scan > 20,
        "raw depth cloud": raw_cloud.width * raw_cloud.height > 100,
        "gmapping grid": grid.info.width * grid.info.height > 0,
        "accumulated 3D cloud": octomap_cloud.width * octomap_cloud.height > 0,
    }
    failed = [name for name, passed in checks.items() if not passed]
    for name, passed in checks.items():
        rospy.loginfo("%s: %s", name, "PASS" if passed else "FAIL")
    if failed:
        rospy.logerr("Failed checks: %s", ", ".join(failed))
        return 1
    rospy.loginfo("Gazebo, TurtleBot3, gmapping, and 3D point cloud are healthy")
    return 0


if __name__ == "__main__":
    sys.exit(main())
