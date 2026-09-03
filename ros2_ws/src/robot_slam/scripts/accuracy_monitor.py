#!/usr/bin/env python3
"""Show live SLAM localization errors against Gazebo ground truth."""

import math
import tkinter as tk
from dataclasses import dataclass

import rclpy  # pyright: ignore[reportMissingImports]
from geometry_msgs.msg import (  # pyright: ignore[reportMissingImports]
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from rclpy.duration import Duration  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from rclpy.time import Time  # pyright: ignore[reportMissingImports]
from tf2_ros import (  # pyright: ignore[reportMissingImports]
    Buffer,
    TransformException,
    TransformListener,
)


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


def compose(left: Pose2D, right: Pose2D):
    cosine = math.cos(left.yaw)
    sine = math.sin(left.yaw)
    return Pose2D(
        left.x + cosine * right.x - sine * right.y,
        left.y + sine * right.x + cosine * right.y,
        normalize_angle(left.yaw + right.yaw),
    )


def inverse(pose: Pose2D):
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Pose2D(
        -cosine * pose.x - sine * pose.y,
        sine * pose.x - cosine * pose.y,
        normalize_angle(-pose.yaw),
    )


class ErrorStats:
    """Accumulate errors after aligning an estimator's initial frame to truth."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.alignment = None
        self.count = 0
        self.position_squared_sum = 0.0
        self.heading_squared_sum = 0.0
        self.current_position = 0.0
        self.current_heading = 0.0

    def update(self, estimate: Pose2D, truth: Pose2D):
        if self.alignment is None:
            self.alignment = compose(truth, inverse(estimate))

        aligned_estimate = compose(self.alignment, estimate)
        self.current_position = math.hypot(
            aligned_estimate.x - truth.x,
            aligned_estimate.y - truth.y,
        )
        self.current_heading = abs(
            normalize_angle(aligned_estimate.yaw - truth.yaw)
        )
        self.count += 1
        self.position_squared_sum += self.current_position**2
        self.heading_squared_sum += self.current_heading**2

    @property
    def position_rmse(self):
        return math.sqrt(self.position_squared_sum / self.count)

    @property
    def heading_rmse(self):
        return math.sqrt(self.heading_squared_sum / self.count)


class AccuracyMonitor(Node):
    def __init__(self):
        super().__init__("accuracy_monitor")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter(
            "base_frame", "base_footprint"
        ).value
        ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/ground_truth/pose"
        ).value
        rbpf_pose_topic = self.declare_parameter(
            "rbpf_pose_topic", "/rbpf/pose"
        ).value

        self.truth = None
        self.truth_stamp = None
        self.last_sampled_truth_stamp = None
        self.rbpf_pose = None
        self.slam_stats = ErrorStats()
        self.rbpf_stats = ErrorStats()

        self.create_subscription(
            PoseStamped, ground_truth_topic, self._truth_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            rbpf_pose_topic,
            self._rbpf_callback,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    @staticmethod
    def _pose_from_message(pose: Pose):
        return Pose2D(
            pose.position.x,
            pose.position.y,
            quaternion_yaw(pose.orientation),
        )

    def _truth_callback(self, message: PoseStamped):
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        if self.truth_stamp is not None and stamp < self.truth_stamp:
            self.slam_stats.reset()
            self.rbpf_stats.reset()
            self.last_sampled_truth_stamp = None
            self.rbpf_pose = None
        self.truth_stamp = stamp
        self.truth = self._pose_from_message(message.pose)

    def _rbpf_callback(self, message: PoseWithCovarianceStamped):
        self.rbpf_pose = self._pose_from_message(message.pose.pose)

    def _slam_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        return Pose2D(
            translation.x,
            translation.y,
            quaternion_yaw(transform.transform.rotation),
        )

    def sample(self):
        if (
            self.truth is None
            or self.truth_stamp == self.last_sampled_truth_stamp
        ):
            return

        slam_pose = self._slam_pose()
        if slam_pose is not None:
            self.slam_stats.update(slam_pose, self.truth)
        if self.rbpf_pose is not None:
            self.rbpf_stats.update(self.rbpf_pose, self.truth)
        self.last_sampled_truth_stamp = self.truth_stamp


class AccuracyWindow:
    def __init__(self, monitor: AccuracyMonitor):
        self.monitor = monitor
        self.root = tk.Tk(className="RobotSlamAccuracy")
        self.root.title("SLAM accuracy")
        self.root.geometry("540x190+20+20")
        self.root.resizable(False, False)
        self.root.configure(background="#20242b")
        self.root.attributes("-topmost", True)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        tk.Label(
            self.root,
            text="LIVE LOCALIZATION ACCURACY",
            background="#20242b",
            foreground="#f2f5f7",
            font=("DejaVu Sans", 14, "bold"),
        ).pack(pady=(12, 1))
        tk.Label(
            self.root,
            text="Against Gazebo ground truth  •  lower is better",
            background="#20242b",
            foreground="#aeb8c4",
            font=("DejaVu Sans", 9),
        ).pack(pady=(0, 8))

        self.slam_text = tk.StringVar(value="Waiting for SLAM Toolbox pose…")
        self.rbpf_text = tk.StringVar(value="Waiting for RBPF pose…")
        self._metric_row("SLAM Toolbox", self.slam_text, "#4fc3f7")
        self._metric_row("RBPF", self.rbpf_text, "#ffb74d")

        tk.Label(
            self.root,
            text="Position / heading: current error  |  running RMSE",
            background="#20242b",
            foreground="#7f8b98",
            font=("DejaVu Sans", 8),
        ).pack(pady=(7, 0))

    def _metric_row(self, name: str, variable: tk.StringVar, color: str):
        row = tk.Frame(self.root, background="#20242b")
        row.pack(fill="x", padx=18, pady=3)
        tk.Label(
            row,
            text=name,
            width=14,
            anchor="w",
            background="#20242b",
            foreground=color,
            font=("DejaVu Sans", 10, "bold"),
        ).pack(side="left")
        tk.Label(
            row,
            textvariable=variable,
            anchor="w",
            background="#20242b",
            foreground="#edf1f5",
            font=("DejaVu Sans Mono", 9),
        ).pack(side="left")

    @staticmethod
    def _format(stats: ErrorStats):
        if stats.count == 0:
            return "Waiting for estimator pose…"
        current_degrees = math.degrees(stats.current_heading)
        rmse_degrees = math.degrees(stats.heading_rmse)
        return (
            f"{stats.current_position:5.3f} m / {current_degrees:4.1f}°"
            f"  |  {stats.position_rmse:5.3f} m / {rmse_degrees:4.1f}°"
        )

    def _update(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        # A single spin only executes one callback. Drain a small batch so the
        # 30 Hz truth and TF streams cannot backlog behind the 10 Hz UI refresh.
        for _ in range(20):
            rclpy.spin_once(self.monitor, timeout_sec=0.0)
        self.monitor.sample()
        self.slam_text.set(self._format(self.monitor.slam_stats))
        self.rbpf_text.set(self._format(self.monitor.rbpf_stats))
        self.root.after(100, self._update)

    def _close(self):
        self.root.destroy()

    def run(self):
        self.root.after(0, self._update)
        self.root.mainloop()


def main():
    rclpy.init()
    monitor = AccuracyMonitor()
    try:
        AccuracyWindow(monitor).run()
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
