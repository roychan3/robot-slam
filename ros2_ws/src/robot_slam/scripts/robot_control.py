#!/usr/bin/env python3
"""Provide a compact, hold-to-drive control panel for the browser UI."""

from functools import partial
import tkinter as tk

import rclpy  # pyright: ignore[reportMissingImports]
from geometry_msgs.msg import Twist  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]


class RobotController(Node):
    """Publish velocity commands for the simulated robot."""

    def __init__(self):
        super().__init__("robot_control")
        cmd_vel_topic = str(
            self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        )
        self.linear_speed = float(
            self.declare_parameter("linear_speed", 0.22).value
        )
        self.angular_speed = float(
            self.declare_parameter("angular_speed", 0.80).value
        )
        self.max_linear_speed = float(
            self.declare_parameter("max_linear_speed", 0.26).value
        )
        self.max_angular_speed = float(
            self.declare_parameter("max_angular_speed", 1.82).value
        )
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

    def drive(self, linear: float, angular: float):
        linear = max(
            -self.max_linear_speed, min(self.max_linear_speed, linear)
        )
        angular = max(
            -self.max_angular_speed, min(self.max_angular_speed, angular)
        )
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.publisher.publish(command)

    def stop(self):
        self.drive(0.0, 0.0)


class ControlWindow:
    """Render the controls and translate held buttons or keys into motion."""

    _MIN_LINEAR_SPEED = 0.05
    _MIN_ANGULAR_SPEED = 0.20
    _KEY_ACTIONS = {
        "w": "forward",
        "up": "forward",
        "s": "reverse",
        "down": "reverse",
        "a": "left",
        "left": "left",
        "d": "right",
        "right": "right",
    }

    def __init__(self, controller: RobotController):
        self.controller = controller
        self.held_actions: set[str] = set()
        self.closed = False

        self.root = tk.Tk(className="RobotSlamControl")
        self.root.title("Robot controls")
        self.root.geometry("680x220+580+20")
        self.root.resizable(False, False)
        self.root.configure(background="#20242b")
        self.root.attributes("-topmost", True)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self.linear_speed = tk.DoubleVar(value=controller.linear_speed)
        self.angular_speed = tk.DoubleVar(value=controller.angular_speed)
        self.status = tk.StringVar(value="STOPPED")

        self._build_panel()
        self.root.bind("<KeyPress>", self._key_press)
        self.root.bind("<KeyRelease>", self._key_release)
        self.root.bind("<FocusOut>", self._focus_out)

    def _build_panel(self):
        tk.Label(
            self.root,
            text="ROBOT CONTROL",
            background="#20242b",
            foreground="#f2f5f7",
            font=("DejaVu Sans", 14, "bold"),
        ).pack(pady=(8, 0))
        tk.Label(
            self.root,
            text="Hold a button or use WASD / arrow keys",
            background="#20242b",
            foreground="#aeb8c4",
            font=("DejaVu Sans", 9),
        ).pack(pady=(0, 4))

        content = tk.Frame(self.root, background="#20242b")
        content.pack(fill="both", expand=True, padx=14, pady=(0, 8))

        pad = tk.Frame(content, background="#20242b")
        pad.pack(side="left", anchor="n")
        self._movement_button(pad, "▲\nFORWARD", "forward", 0, 1)
        self._movement_button(pad, "◀\nLEFT", "left", 1, 0)
        stop = tk.Button(
            pad,
            text="■\nSTOP",
            width=8,
            height=2,
            background="#d9534f",
            activebackground="#ef6c67",
            foreground="white",
            activeforeground="white",
            relief="flat",
            font=("DejaVu Sans", 9, "bold"),
            command=self._stop,
            takefocus=True,
        )
        stop.grid(row=1, column=1, padx=3, pady=2)
        self._movement_button(pad, "▶\nRIGHT", "right", 1, 2)
        self._movement_button(pad, "▼\nREVERSE", "reverse", 2, 1)

        speeds = tk.Frame(content, background="#20242b")
        speeds.pack(
            side="left", fill="both", expand=True, padx=(14, 0), pady=(4, 0)
        )
        self._speed_control(
            speeds,
            "Linear speed",
            self.linear_speed,
            self._MIN_LINEAR_SPEED,
            self.controller.max_linear_speed,
            "m/s",
            0,
        )
        self._speed_control(
            speeds,
            "Turn speed",
            self.angular_speed,
            self._MIN_ANGULAR_SPEED,
            self.controller.max_angular_speed,
            "rad/s",
            1,
        )

        tk.Label(
            speeds,
            textvariable=self.status,
            background="#15191e",
            foreground="#81c784",
            font=("DejaVu Sans Mono", 10, "bold"),
            pady=6,
        ).grid(row=2, column=0, columnspan=3, sticky="ew", pady=(12, 5))
        tk.Label(
            speeds,
            text="Space or STOP halts movement immediately",
            background="#20242b",
            foreground="#7f8b98",
            font=("DejaVu Sans", 8),
        ).grid(row=3, column=0, columnspan=3)

    def _movement_button(
        self,
        parent: tk.Frame,
        label: str,
        action: str,
        row: int,
        column: int,
    ):
        button = tk.Button(
            parent,
            text=label,
            width=8,
            height=2,
            background="#3c4652",
            activebackground="#4fc3f7",
            foreground="#f2f5f7",
            activeforeground="#15202a",
            relief="flat",
            font=("DejaVu Sans", 9, "bold"),
            takefocus=True,
        )
        button.grid(row=row, column=column, padx=3, pady=2)
        button.bind(
            "<ButtonPress-1>", partial(self._button_press, action)
        )
        button.bind(
            "<ButtonRelease-1>", partial(self._button_release, action)
        )

    def _speed_control(
        self,
        parent: tk.Frame,
        label: str,
        variable: tk.DoubleVar,
        minimum: float,
        maximum: float,
        unit: str,
        row: int,
    ):
        tk.Label(
            parent,
            text=label,
            width=11,
            anchor="w",
            background="#20242b",
            foreground="#d9e0e7",
            font=("DejaVu Sans", 9),
        ).grid(row=row, column=0, sticky="w")
        tk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            resolution=0.01,
            orient="horizontal",
            showvalue=False,
            variable=variable,
            command=self._speed_changed,
            length=130,
            sliderlength=16,
            highlightthickness=0,
            background="#20242b",
            troughcolor="#11151a",
            activebackground="#4fc3f7",
        ).grid(row=row, column=1, padx=5)
        value_label = tk.Label(
            parent,
            width=9,
            anchor="e",
            background="#20242b",
            foreground="#aeb8c4",
            font=("DejaVu Sans Mono", 8),
        )
        value_label.grid(row=row, column=2)

        def refresh_value(*_args: object):
            value_label.configure(text=f"{variable.get():.2f} {unit}")

        variable.trace_add("write", refresh_value)
        refresh_value()

    def _button_press(self, action: str, _event: tk.Event):
        self.held_actions.add(action)
        self._publish_current()

    def _button_release(self, action: str, _event: tk.Event):
        self.held_actions.discard(action)
        self._publish_current()

    def _key_press(self, event: tk.Event):
        key = event.keysym.lower()
        if key == "space":
            self._stop()
            return "break"
        action = self._KEY_ACTIONS.get(key)
        if action is not None:
            self.held_actions.add(action)
            self._publish_current()
            return "break"
        return None

    def _key_release(self, event: tk.Event):
        action = self._KEY_ACTIONS.get(event.keysym.lower())
        if action is not None:
            self.held_actions.discard(action)
            self._publish_current()
            return "break"
        return None

    def _focus_out(self, event: tk.Event):
        # Focus moving between widgets inside this window is harmless. Stop if
        # the operator switches to Gazebo, RViz, or another browser window.
        self.root.after_idle(self._stop_if_unfocused)

    def _stop_if_unfocused(self):
        if self.root.focus_displayof() is None:
            self._stop()

    def _speed_changed(self, _value: str):
        if self.held_actions:
            self._publish_current()

    def _stop(self):
        self.held_actions.clear()
        self.controller.stop()
        self.status.set("STOPPED")

    def _publish_current(self):
        forward = int("forward" in self.held_actions)
        reverse = int("reverse" in self.held_actions)
        left = int("left" in self.held_actions)
        right = int("right" in self.held_actions)
        linear_speed = max(
            self._MIN_LINEAR_SPEED, self.linear_speed.get()
        )
        angular_speed = max(
            self._MIN_ANGULAR_SPEED, self.angular_speed.get()
        )
        linear = (forward - reverse) * linear_speed
        angular = (left - right) * angular_speed
        self.controller.drive(linear, angular)

        if linear == 0.0 and angular == 0.0:
            self.status.set("STOPPED")
        else:
            self.status.set(
                f"LINEAR {linear:+.2f} m/s   TURN {angular:+.2f} rad/s"
            )

    def _update(self):
        if not rclpy.ok():
            self._close()
            return
        for _ in range(5):
            rclpy.spin_once(self.controller, timeout_sec=0.0)
        if self.held_actions:
            # Keep commands fresh while a control is held. Releasing any
            # control immediately publishes the new command (or a full stop).
            self._publish_current()
        self.root.after(100, self._update)

    def _close(self):
        if self.closed:
            return
        self.closed = True
        if rclpy.ok():
            self.controller.stop()
        self.root.destroy()

    def run(self):
        self.root.after(0, self._update)
        self.root.mainloop()


def main():
    rclpy.init()
    controller = RobotController()
    try:
        ControlWindow(controller).run()
    finally:
        if rclpy.ok():
            controller.stop()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
