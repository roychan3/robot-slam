#!/usr/bin/env python3
"""Save one ROS 2 PointCloud2 message as an ASCII PCD file."""

import argparse
import math
import os
import sys

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.utilities import remove_ros_args  # pyright: ignore[reportMissingImports]
from rclpy.wait_for_message import (  # pyright: ignore[reportMissingImports]
    wait_for_message,
)
from sensor_msgs.msg import PointCloud2  # pyright: ignore[reportMissingImports]
from sensor_msgs_py import point_cloud2  # pyright: ignore[reportMissingImports]


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
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("save_pointcloud")

    try:
        received, cloud = wait_for_message(
            PointCloud2, node, args.topic, time_to_wait=args.timeout
        )
        if not received:
            node.get_logger().error(
                f"No point cloud received from {args.topic} within "
                f"{args.timeout:.1f} seconds"
            )
            return 1

        points = []
        for point in point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        ):
            xyz = (float(point["x"]), float(point["y"]), float(point["z"]))
            if all(math.isfinite(value) for value in xyz):
                points.append(xyz)
        if not points:
            node.get_logger().error(
                f"Point cloud {args.topic} contained no finite XYZ points"
            )
            return 1

        output = os.path.abspath(args.output)
        os.makedirs(os.path.dirname(output), exist_ok=True)
        with open(output, "w", encoding="ascii") as pcd:
            pcd.write("# .PCD v0.7 - Point Cloud Data file format\n")
            pcd.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n")
            pcd.write("TYPE F F F\nCOUNT 1 1 1\n")
            pcd.write(f"WIDTH {len(points)}\nHEIGHT 1\n")
            pcd.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            pcd.write(f"POINTS {len(points)}\nDATA ascii\n")
            for x, y, z in points:
                pcd.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        node.get_logger().info(
            f"Saved {len(points)} points from {args.topic} to {output}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
