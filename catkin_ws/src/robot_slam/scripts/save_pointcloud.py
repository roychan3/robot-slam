#!/usr/bin/env python3
"""Save one ROS PointCloud2 message as an ASCII PCD file."""

import argparse
import math
import os
import sys

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


def parse_args():
    parser = argparse.ArgumentParser(
        description="Save the latest accumulated OctoMap point cloud to PCD."
    )
    parser.add_argument(
        "--topic", default="/octomap_point_cloud_centers", help="PointCloud2 topic"
    )
    parser.add_argument(
        "--output", default="/data/indoor_environment.pcd", help="Output PCD path"
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    args = parse_args()
    rospy.init_node("save_pointcloud", anonymous=True, disable_signals=True)

    try:
        cloud = rospy.wait_for_message(args.topic, PointCloud2, timeout=args.timeout)
    except rospy.ROSException as exc:
        rospy.logerr("No point cloud received from %s: %s", args.topic, exc)
        return 1

    points = [
        (x, y, z)
        for x, y, z in point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
    ]
    if not points:
        rospy.logerr("Point cloud %s contained no finite XYZ points", args.topic)
        return 1

    output = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="ascii") as pcd:
        pcd.write("# .PCD v0.7 - Point Cloud Data file format\n")
        pcd.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n")
        pcd.write("TYPE F F F\nCOUNT 1 1 1\n")
        pcd.write("WIDTH {}\nHEIGHT 1\n".format(len(points)))
        pcd.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        pcd.write("POINTS {}\nDATA ascii\n".format(len(points)))
        for x, y, z in points:
            pcd.write("{:.6f} {:.6f} {:.6f}\n".format(x, y, z))

    rospy.loginfo("Saved %d points from %s to %s", len(points), args.topic, output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
