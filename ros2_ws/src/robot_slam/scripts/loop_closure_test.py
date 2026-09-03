#!/usr/bin/env python3
"""Drive one closed circuit and verify odometry and SLAM loop closure."""

import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import TypeVar, cast

import rclpy  # pyright: ignore[reportMissingImports]
from geometry_msgs.msg import (  # pyright: ignore[reportMissingImports]
    Quaternion,
    Twist,
)
from nav_msgs.msg import (  # pyright: ignore[reportMissingImports]
    OccupancyGrid,
    Odometry,
)
from rclpy.duration import Duration  # pyright: ignore[reportMissingImports]
from rclpy.executors import (  # pyright: ignore[reportMissingImports]
    MultiThreadedExecutor,
)
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from rclpy.parameter import Parameter  # pyright: ignore[reportMissingImports]
from rclpy.qos import (  # pyright: ignore[reportMissingImports]
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time  # pyright: ignore[reportMissingImports]
from sensor_msgs.msg import LaserScan  # pyright: ignore[reportMissingImports]
from tf2_ros import (  # pyright: ignore[reportMissingImports]
    Buffer,
    TransformException,
    TransformListener,
)

_T = TypeVar("_T")


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def normalize_angle(angle: float):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion: Quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, lower: float, upper: float):
    return max(lower, min(upper, value))


class LoopClosureTest(Node):
    def __init__(self):
        super().__init__("robot_slam_loop_closure_test")
        self.set_parameters([Parameter("use_sim_time", value=True)])

        self.cmd_vel_topic = self._parameter("cmd_vel_topic", "/cmd_vel")
        self.odom_topic = self._parameter("odom_topic", "/odom")
        self.scan_topic = self._parameter("scan_topic", "/scan")
        self.map_frame = self._parameter("map_frame", "map")
        self.base_frame = self._parameter("base_frame", "base_footprint")

        self.loop_width = float(self._parameter("loop_width", 2.0))
        self.loop_height = float(self._parameter("loop_height", 1.5))
        self.linear_speed = float(self._parameter("linear_speed", 0.22))
        self.angular_speed = float(self._parameter("angular_speed", 0.65))
        self.waypoint_tolerance = float(
            self._parameter("waypoint_tolerance", 0.07)
        )
        self.heading_tolerance = float(
            self._parameter("heading_tolerance", 0.04)
        )
        self.position_tolerance = float(
            self._parameter("position_tolerance", 0.20)
        )
        self.yaw_tolerance = float(self._parameter("yaw_tolerance", 0.20))
        self.slam_position_tolerance = float(
            self._parameter("slam_position_tolerance", 0.30)
        )
        self.slam_yaw_tolerance = float(
            self._parameter("slam_yaw_tolerance", 0.25)
        )
        self.obstacle_distance = float(
            self._parameter("obstacle_distance", 0.38)
        )
        self.motion_timeout = float(self._parameter("motion_timeout", 150.0))
        self.startup_timeout = float(self._parameter("startup_timeout", 75.0))
        self.scan_timeout = float(self._parameter("scan_timeout", 3.0))

        if self.loop_width <= 0.0 or self.loop_height <= 0.0:
            raise ValueError("loop_width and loop_height must be positive")

        self._lock = threading.Lock()
        self._odom_pose = None
        self._last_path_pose = None
        self._path_length = 0.0
        self._front_clearance = float("inf")
        self._last_scan_time = None
        self._map_received = False

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 1)
        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, sensor_qos
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, map_qos
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _parameter(self, name: str, default: _T) -> _T:
        return cast(_T, self.declare_parameter(name, default).value)

    def _odom_callback(self, message: Odometry):
        pose = Pose2D(
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            quaternion_yaw(message.pose.pose.orientation),
        )
        with self._lock:
            if self._last_path_pose is not None:
                step = math.hypot(
                    pose.x - self._last_path_pose.x,
                    pose.y - self._last_path_pose.y,
                )
                if step < 0.5:
                    self._path_length += step
            self._last_path_pose = pose
            self._odom_pose = pose

    def _scan_callback(self, message: LaserScan):
        half_angle = math.radians(20.0)
        clearances = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(normalize_angle(angle)) <= half_angle:
                if math.isfinite(distance) and message.range_min < distance:
                    clearances.append(distance)
        with self._lock:
            self._front_clearance = min(clearances, default=float("inf"))
            self._last_scan_time = self.get_clock().now()

    def _map_callback(self, _message: OccupancyGrid):
        with self._lock:
            self._map_received = True

    def _current_odom(self):
        with self._lock:
            return self._odom_pose

    def _current_path_length(self):
        with self._lock:
            return self._path_length

    def _current_front_clearance(self):
        with self._lock:
            if self._last_scan_time is None:
                return None
            age = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
            if age > self.scan_timeout:
                return None
            return self._front_clearance

    def _reset_path_length(self):
        with self._lock:
            self._path_length = 0.0
            self._last_path_pose = self._odom_pose

    def _slam_pose(self):
        transform = self._tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return Pose2D(translation.x, translation.y, quaternion_yaw(rotation))

    def _wait_for_inputs(self):
        self.get_logger().info(
            "Waiting for odometry, lidar, map, and SLAM transform"
        )
        deadline = time.monotonic() + self.startup_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                ready = (
                    self._odom_pose is not None
                    and self._last_scan_time is not None
                    and self._map_received
                )
            if ready:
                try:
                    return self._slam_pose()
                except TransformException:
                    pass
            time.sleep(0.1)
        raise RuntimeError("odometry, lidar, map, or SLAM transform was unavailable")

    def stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_velocity(self, linear: float = 0.0, angular: float = 0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._cmd_pub.publish(command)

    def _rotate_to(self, target_yaw: float, deadline: Time):
        rate = self.create_rate(20.0)
        while rclpy.ok():
            if self.get_clock().now() >= deadline:
                raise RuntimeError("motion timed out while rotating")
            pose = self._current_odom()
            if pose is None:
                rate.sleep()
                continue
            error = normalize_angle(target_yaw - pose.yaw)
            if abs(error) <= self.heading_tolerance:
                self.stop()
                return
            angular = clamp(1.8 * error, -self.angular_speed, self.angular_speed)
            if abs(angular) < 0.12:
                angular = math.copysign(0.12, angular)
            self._publish_velocity(angular=angular)
            rate.sleep()
        raise RuntimeError("ROS shut down while rotating")

    def _drive_to(self, goal_x: float, goal_y: float, deadline: Time):
        rate = self.create_rate(20.0)
        while rclpy.ok():
            if self.get_clock().now() >= deadline:
                raise RuntimeError("motion timed out while driving")
            pose = self._current_odom()
            if pose is None:
                rate.sleep()
                continue

            dx = goal_x - pose.x
            dy = goal_y - pose.y
            distance = math.hypot(dx, dy)
            if distance <= self.waypoint_tolerance:
                self.stop()
                return

            target_yaw = math.atan2(dy, dx)
            heading_error = normalize_angle(target_yaw - pose.yaw)
            if abs(heading_error) > 0.60:
                linear = 0.0
            else:
                linear = min(self.linear_speed, max(0.06, 0.7 * distance))
                linear *= max(0.25, 1.0 - abs(heading_error) / 0.60)

            if linear > 0.0:
                clearance = self._current_front_clearance()
                if clearance is None:
                    raise RuntimeError(
                        f"no lidar data for over {self.scan_timeout:.1f} s; "
                        "refusing to drive"
                    )
                if clearance < self.obstacle_distance:
                    raise RuntimeError(
                        f"obstacle detected {clearance:.2f} m ahead"
                    )

            angular = clamp(
                1.8 * heading_error, -self.angular_speed, self.angular_speed
            )
            self._publish_velocity(linear=linear, angular=angular)
            rate.sleep()
        raise RuntimeError("ROS shut down while driving")

    def _world_waypoints(self, start: Pose2D):
        local_waypoints = (
            (self.loop_width, 0.0),
            (self.loop_width, self.loop_height),
            (0.0, self.loop_height),
            (0.0, 0.0),
        )
        cosine = math.cos(start.yaw)
        sine = math.sin(start.yaw)
        return [
            (
                start.x + cosine * x - sine * y,
                start.y + sine * x + cosine * y,
            )
            for x, y in local_waypoints
        ]

    def run(self):
        initial_slam = self._wait_for_inputs()
        initial_odom = self._current_odom()
        if initial_odom is None:
            raise RuntimeError("odometry was unavailable after startup")

        self._reset_path_length()
        waypoints = self._world_waypoints(initial_odom)
        expected_path_length = 2.0 * (self.loop_width + self.loop_height)
        self.get_logger().info(
            f"Starting {self.loop_width:.2f} m x {self.loop_height:.2f} m loop "
            f"at odom ({initial_odom.x:.2f}, {initial_odom.y:.2f}, "
            f"{initial_odom.yaw:.2f} rad)"
        )

        # Keep this budget in simulated time so a low real-time factor does not
        # reduce how far the robot is allowed to travel.
        deadline = self.get_clock().now() + Duration(seconds=self.motion_timeout)
        try:
            for index, (goal_x, goal_y) in enumerate(waypoints, start=1):
                pose = self._current_odom()
                if pose is None:
                    raise RuntimeError("odometry stopped publishing mid-loop")
                target_yaw = math.atan2(goal_y - pose.y, goal_x - pose.x)
                self.get_logger().info(
                    f"Waypoint {index}/{len(waypoints)}: "
                    f"({goal_x:.2f}, {goal_y:.2f})"
                )
                self._rotate_to(target_yaw, deadline)
                self._drive_to(goal_x, goal_y, deadline)
            self._rotate_to(initial_odom.yaw, deadline)
        finally:
            self.stop()

        settle_rate = self.create_rate(0.5)
        settle_rate.sleep()
        final_odom = self._current_odom()
        final_slam = self._slam_pose()
        path_length = self._current_path_length()
        if final_odom is None:
            raise RuntimeError("odometry was unavailable after the loop")

        odom_position_error = math.hypot(
            final_odom.x - initial_odom.x, final_odom.y - initial_odom.y
        )
        odom_yaw_error = abs(normalize_angle(final_odom.yaw - initial_odom.yaw))
        slam_position_error = math.hypot(
            final_slam.x - initial_slam.x, final_slam.y - initial_slam.y
        )
        slam_yaw_error = abs(normalize_angle(final_slam.yaw - initial_slam.yaw))

        checks = {
            "completed circuit": path_length >= expected_path_length * 0.85,
            "returned in odometry": odom_position_error <= self.position_tolerance,
            "restored odometry heading": odom_yaw_error <= self.yaw_tolerance,
            "returned in SLAM map": (
                slam_position_error <= self.slam_position_tolerance
            ),
            "restored SLAM heading": slam_yaw_error <= self.slam_yaw_tolerance,
        }

        self.get_logger().info(
            f"Loop result: path={path_length:.2f} m, "
            f"odom_error={odom_position_error:.3f} m/{odom_yaw_error:.3f} rad, "
            f"slam_error={slam_position_error:.3f} m/{slam_yaw_error:.3f} rad"
        )
        for name, passed in checks.items():
            self.get_logger().info(f"{name}: {'PASS' if passed else 'FAIL'}")

        failed = [name for name, passed in checks.items() if not passed]
        if failed:
            self.get_logger().error(
                f"LOOP_CLOSURE_RESULT: FAIL ({', '.join(failed)})"
            )
            return 1
        self.get_logger().info("LOOP_CLOSURE_RESULT: PASS")
        return 0


def main():
    rclpy.init()
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    stop_spinning = threading.Event()
    spin_thread = None
    try:
        node = LoopClosureTest()
        executor.add_node(node)

        def spin():
            while rclpy.ok() and not stop_spinning.is_set():
                executor.spin_once(timeout_sec=0.1)

        spin_thread = threading.Thread(target=spin, daemon=True)
        spin_thread.start()
        return node.run()
    except (TransformException, RuntimeError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(f"LOOP_CLOSURE_RESULT: ERROR: {exc}")
        else:
            print(f"LOOP_CLOSURE_RESULT: ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.stop()
        stop_spinning.set()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
