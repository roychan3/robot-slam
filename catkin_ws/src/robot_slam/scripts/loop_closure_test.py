#!/usr/bin/env python3
"""Drive one closed circuit and verify odometry and SLAM loop closure."""

import math
import sys
import threading
import time
from dataclasses import dataclass

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class LoopClosureTest:
    def __init__(self):
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")

        self.loop_width = float(rospy.get_param("~loop_width", 2.0))
        self.loop_height = float(rospy.get_param("~loop_height", 1.5))
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.22))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.65))
        self.waypoint_tolerance = float(
            rospy.get_param("~waypoint_tolerance", 0.07)
        )
        self.heading_tolerance = float(
            rospy.get_param("~heading_tolerance", 0.04)
        )
        self.position_tolerance = float(
            rospy.get_param("~position_tolerance", 0.20)
        )
        self.yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 0.20))
        self.slam_position_tolerance = float(
            rospy.get_param("~slam_position_tolerance", 0.30)
        )
        self.slam_yaw_tolerance = float(
            rospy.get_param("~slam_yaw_tolerance", 0.25)
        )
        self.obstacle_distance = float(
            rospy.get_param("~obstacle_distance", 0.38)
        )
        self.motion_timeout = float(rospy.get_param("~motion_timeout", 150.0))
        self.startup_timeout = float(rospy.get_param("~startup_timeout", 75.0))

        if self.loop_width <= 0.0 or self.loop_height <= 0.0:
            raise ValueError("loop_width and loop_height must be positive")

        self._lock = threading.Lock()
        self._odom_pose = None
        self._last_path_pose = None
        self._path_length = 0.0
        self._front_clearance = float("inf")
        self._last_scan_time = None
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 3.0))

        self._cmd_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=1
        )
        rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=1
        )
        rospy.Subscriber(
            self.scan_topic, LaserScan, self._scan_callback, queue_size=1
        )

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.on_shutdown(self.stop)

    def _odom_callback(self, message):
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
                # Ignore discontinuities caused by simulation resets.
                if step < 0.5:
                    self._path_length += step
            self._last_path_pose = pose
            self._odom_pose = pose

    def _scan_callback(self, message):
        half_angle = math.radians(20.0)
        clearances = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(normalize_angle(angle)) <= half_angle:
                if math.isfinite(distance) and message.range_min < distance:
                    clearances.append(distance)
        with self._lock:
            self._front_clearance = min(clearances, default=float("inf"))
            self._last_scan_time = rospy.Time.now()

    def _current_odom(self):
        with self._lock:
            return self._odom_pose

    def _current_path_length(self):
        with self._lock:
            return self._path_length

    def _current_front_clearance(self):
        """Front clearance in metres, or None when the lidar has gone quiet.

        Returning inf for a dead sensor would read as 'nothing ahead' and
        silently disable the obstacle guard for the rest of the run, so an
        unknown clearance is reported as unknown and the caller aborts.
        """

        with self._lock:
            if self._last_scan_time is None:
                return None
            age = (rospy.Time.now() - self._last_scan_time).to_sec()
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
            rospy.Time(0),
            rospy.Duration(1.0),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return Pose2D(translation.x, translation.y, quaternion_yaw(rotation))

    def _wait_for_inputs(self):
        rospy.loginfo("Waiting for odometry, lidar, map, and SLAM transform")
        rospy.wait_for_message(
            self.odom_topic, Odometry, timeout=self.startup_timeout
        )
        rospy.wait_for_message(
            self.scan_topic, LaserScan, timeout=self.startup_timeout
        )
        rospy.wait_for_message("/map", OccupancyGrid, timeout=self.startup_timeout)

        deadline = time.monotonic() + self.startup_timeout
        while not rospy.is_shutdown():
            try:
                return self._slam_pose()
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "SLAM transform {} -> {} was not available".format(
                            self.map_frame, self.base_frame
                        )
                    )
                rospy.sleep(0.1)
        raise RuntimeError("ROS shut down while waiting for SLAM")

    def stop(self):
        if hasattr(self, "_cmd_pub"):
            self._cmd_pub.publish(Twist())

    def _publish_velocity(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._cmd_pub.publish(command)

    def _rotate_to(self, target_yaw, deadline):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if rospy.Time.now() >= deadline:
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

    def _drive_to(self, goal_x, goal_y, deadline):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if rospy.Time.now() >= deadline:
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
                        "no lidar data for over {:.1f} s; refusing to drive"
                        .format(self.scan_timeout)
                    )
                if clearance < self.obstacle_distance:
                    raise RuntimeError(
                        "obstacle detected {:.2f} m ahead".format(clearance)
                    )

            angular = clamp(
                1.8 * heading_error, -self.angular_speed, self.angular_speed
            )
            self._publish_velocity(linear=linear, angular=angular)
            rate.sleep()
        raise RuntimeError("ROS shut down while driving")

    def _world_waypoints(self, start):
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
        rospy.loginfo(
            "Starting %.2f m x %.2f m loop at odom (%.2f, %.2f, %.2f rad)",
            self.loop_width,
            self.loop_height,
            initial_odom.x,
            initial_odom.y,
            initial_odom.yaw,
        )

        # Simulated time, not wall clock. Rate/sleep in the drive loops run on
        # /clock, so a wall-clock budget silently shrinks with Gazebo's
        # real-time factor: at RTF 0.3 a 150 s budget buys 45 s of robot
        # motion, and the run fails as 'motion timed out' rather than for
        # anything to do with the robot. The startup wait below stays on the
        # wall clock on purpose, so a /clock that never starts is still
        # caught rather than waited on forever.
        deadline = rospy.Time.now() + rospy.Duration(self.motion_timeout)
        try:
            for index, (goal_x, goal_y) in enumerate(waypoints, start=1):
                pose = self._current_odom()
                if pose is None:
                    raise RuntimeError("odometry stopped publishing mid-loop")
                target_yaw = math.atan2(goal_y - pose.y, goal_x - pose.x)
                rospy.loginfo(
                    "Waypoint %d/%d: (%.2f, %.2f)",
                    index,
                    len(waypoints),
                    goal_x,
                    goal_y,
                )
                self._rotate_to(target_yaw, deadline)
                self._drive_to(goal_x, goal_y, deadline)
            self._rotate_to(initial_odom.yaw, deadline)
        finally:
            self.stop()

        # Let the final scan and map-to-odom correction settle before measuring.
        rospy.sleep(2.0)
        final_odom = self._current_odom()
        final_slam = self._slam_pose()
        path_length = self._current_path_length()

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

        rospy.loginfo(
            "Loop result: path=%.2f m, odom_error=%.3f m/%.3f rad, "
            "slam_error=%.3f m/%.3f rad",
            path_length,
            odom_position_error,
            odom_yaw_error,
            slam_position_error,
            slam_yaw_error,
        )
        for name, passed in checks.items():
            rospy.loginfo("%s: %s", name, "PASS" if passed else "FAIL")

        failed = [name for name, passed in checks.items() if not passed]
        if failed:
            rospy.logerr("LOOP_CLOSURE_RESULT: FAIL (%s)", ", ".join(failed))
            return 1
        rospy.loginfo("LOOP_CLOSURE_RESULT: PASS")
        return 0


def main():
    rospy.init_node("robot_slam_loop_closure_test")
    try:
        return LoopClosureTest().run()
    except (rospy.ROSException, RuntimeError, ValueError) as exc:
        rospy.logerr("LOOP_CLOSURE_RESULT: ERROR: %s", exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
