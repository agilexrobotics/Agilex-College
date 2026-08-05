#!/usr/bin/env python3

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List

import rclpy
from geometry_msgs.msg import Pose
from piper_msgs.msg import PosCmd
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


INIT_POSITION = {
    "x": 0.055142,
    "y": -0.002581,
    "z": 0.217777,
}

INIT_ORIENTATION_QUAT = {
    "x": 0.0009725635585632747,
    "y": 0.6530434831441294,
    "z": -0.029481433351552566,
    "w": 0.7567457355880148,
}


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def quaternion_to_euler_xyz(x: float, y: float, z: float, w: float) -> Dict[str, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return {"roll": roll, "pitch": pitch, "yaw": yaw}


INIT_EULER = quaternion_to_euler_xyz(**INIT_ORIENTATION_QUAT)


@dataclass(frozen=True)
class SliderSpec:
    key: str
    label: str
    minimum: float
    maximum: float
    resolution: float


class PiperPosSliderNode(Node):
    def __init__(self) -> None:
        super().__init__("piper_pos_slider")

        self.declare_parameter("topic", "pos_cmd")
        self.declare_parameter("feedback_topic", "end_pose")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("x_min", -0.10)
        self.declare_parameter("x_max", 0.35)
        self.declare_parameter("y_min", -0.25)
        self.declare_parameter("y_max", 0.25)
        self.declare_parameter("z_min", 0.21)
        self.declare_parameter("z_max", 0.41)
        self.declare_parameter("angle_min", -3.14)
        self.declare_parameter("angle_max", 3.14)
        self.declare_parameter("gripper", 0.0)
        self.declare_parameter("mode1", 0)
        self.declare_parameter("mode2", 0)

        self.topic = self.get_parameter("topic").value
        self.feedback_topic = self.get_parameter("feedback_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.publish_rate_hz = clamp(self.publish_rate_hz, 1.0, 100.0)
        self.publish_period_sec = 1.0 / self.publish_rate_hz

        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.gripper = float(self.get_parameter("gripper").value)
        self.mode1 = int(self.get_parameter("mode1").value)
        self.mode2 = int(self.get_parameter("mode2").value)

        if self.z_min < 0.21 or self.z_max > 0.41:
            self.get_logger().warn("z range is restricted to [0.21, 0.41] for this test node")
            self.z_min = max(self.z_min, 0.21)
            self.z_max = min(self.z_max, 0.41)

        self.publisher = self.create_publisher(PosCmd, self.topic, 10)
        self.feedback_pose = None
        self.create_subscription(Pose, self.feedback_topic, self._feedback_callback, 10)
        self.values = {
            "x": clamp(INIT_POSITION["x"], self.x_min, self.x_max),
            "y": clamp(INIT_POSITION["y"], self.y_min, self.y_max),
            "z": clamp(INIT_POSITION["z"], self.z_min, self.z_max),
            "roll": clamp(INIT_EULER["roll"], self.angle_min, self.angle_max),
            "yaw": clamp(INIT_EULER["yaw"], self.angle_min, self.angle_max),
            "pitch": clamp(INIT_EULER["pitch"], self.angle_min, self.angle_max),
        }

        self.slider_specs = [
            SliderSpec("x", "x (m)", self.x_min, self.x_max, 0.001),
            SliderSpec("y", "y (m)", self.y_min, self.y_max, 0.001),
            SliderSpec("z", "z (m)", self.z_min, self.z_max, 0.001),
            SliderSpec("roll", "roll (rad)", self.angle_min, self.angle_max, 0.01),
            SliderSpec("yaw", "yaw (rad)", self.angle_min, self.angle_max, 0.01),
            SliderSpec("pitch", "pitch (rad)", self.angle_min, self.angle_max, 0.01),
        ]

        self.get_logger().info(
            "Publishing PosCmd to %s at %.1f Hz; z is limited to %.3f..%.3f m"
            % (self.topic, self.publish_rate_hz, self.z_min, self.z_max)
        )

    def _feedback_callback(self, pose: Pose) -> None:
        self.feedback_pose = pose

    def set_value(self, key: str, value: float) -> None:
        ranges = {
            "x": (self.x_min, self.x_max),
            "y": (self.y_min, self.y_max),
            "z": (self.z_min, self.z_max),
            "roll": (self.angle_min, self.angle_max),
            "yaw": (self.angle_min, self.angle_max),
            "pitch": (self.angle_min, self.angle_max),
        }
        minimum, maximum = ranges[key]
        self.values[key] = clamp(float(value), minimum, maximum)

    def make_message(self) -> PosCmd:
        msg = PosCmd()
        msg.x = self.values["x"]
        msg.y = self.values["y"]
        msg.z = clamp(self.values["z"], self.z_min, self.z_max)
        msg.roll = self.values["roll"]
        msg.pitch = self.values["pitch"]
        msg.yaw = self.values["yaw"]
        msg.gripper = self.gripper
        msg.mode1 = self.mode1
        msg.mode2 = self.mode2
        return msg

    def publish_current(self) -> None:
        self.publisher.publish(self.make_message())

    def seed_from_feedback_pose(self, timeout_sec: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self.feedback_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.feedback_pose is None:
            self.get_logger().warn(
                "No feedback pose received from %s; using spec initialization pose"
                % self.feedback_topic
            )
            return False

        pose = self.feedback_pose
        euler = quaternion_to_euler_xyz(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.set_value("x", pose.position.x)
        self.set_value("y", pose.position.y)
        self.set_value("z", pose.position.z)
        self.set_value("roll", euler["roll"])
        self.set_value("pitch", euler["pitch"])
        self.set_value("yaw", euler["yaw"])
        self.get_logger().info(
            "Seeded x/y/rpy from %s; test will only change commanded z" % self.feedback_topic
        )
        return True

    def run_test_z_motion(self, z_values: Iterable[float], hold_sec: float) -> None:
        self.seed_from_feedback_pose()
        safe_z_values = [clamp(float(z), self.z_min, self.z_max) for z in z_values]
        self.get_logger().info(
            "Starting z-only test motion with values: %s"
            % ", ".join(f"{z:.3f}" for z in safe_z_values)
        )

        for z in safe_z_values:
            self.set_value("z", z)
            deadline = time.monotonic() + hold_sec
            self.get_logger().info("Publishing z=%.3f for %.2f s" % (z, hold_sec))
            while rclpy.ok() and time.monotonic() < deadline:
                self.publish_current()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(self.publish_period_sec)

        self.get_logger().info("z-only test motion finished")


class PiperPosSliderGui:
    def __init__(self, node: PiperPosSliderNode) -> None:
        self.node = node
        self.tk = self._import_tkinter()
        self.root = self.tk.Tk()
        self.root.title("Piper End Pose")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.value_labels = {}
        self.variables = {}
        self.closed = False

        self._build()
        self._schedule_publish()

    def _import_tkinter(self):
        try:
            import tkinter as tk
            from tkinter import ttk
        except ImportError as exc:
            raise RuntimeError(
                "tkinter is required for slider mode. Use --test-z-motion for headless testing."
            ) from exc

        tk.ttk = ttk
        return tk

    def _build(self) -> None:
        ttk = self.tk.ttk
        self.root.columnconfigure(0, weight=1)

        main = ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(1, weight=1)

        for row, spec in enumerate(self.node.slider_specs):
            label = ttk.Label(main, text=spec.label, width=12)
            label.grid(row=row, column=0, sticky="w", padx=(0, 8), pady=5)

            variable = self.tk.DoubleVar(value=self.node.values[spec.key])
            self.variables[spec.key] = variable
            scale = ttk.Scale(
                main,
                from_=spec.minimum,
                to=spec.maximum,
                variable=variable,
                command=self._make_slider_callback(spec),
            )
            scale.grid(row=row, column=1, sticky="ew", padx=(0, 8), pady=5)

            value_label = ttk.Label(main, text=self._format_value(spec.key), width=10)
            value_label.grid(row=row, column=2, sticky="e", pady=5)
            self.value_labels[spec.key] = value_label

        status = ttk.Label(
            main,
            text=(
                "topic: %s | rate: %.1f Hz | z limit: %.2f..%.2f m"
                % (
                    self.node.topic,
                    self.node.publish_rate_hz,
                    self.node.z_min,
                    self.node.z_max,
                )
            ),
        )
        status.grid(row=len(self.node.slider_specs), column=0, columnspan=3, sticky="w", pady=(10, 0))

    def _make_slider_callback(self, spec: SliderSpec) -> Callable[[str], None]:
        def callback(raw_value: str) -> None:
            value = round(float(raw_value) / spec.resolution) * spec.resolution
            self.node.set_value(spec.key, value)
            self.value_labels[spec.key].configure(text=self._format_value(spec.key))

        return callback

    def _format_value(self, key: str) -> str:
        value = self.node.values[key]
        if key in {"x", "y", "z"}:
            return f"{value:.3f}"
        return f"{value:.2f}"

    def _schedule_publish(self) -> None:
        if self.closed:
            return

        self.node.publish_current()
        rclpy.spin_once(self.node, timeout_sec=0.0)
        delay_ms = max(1, int(1000.0 / self.node.publish_rate_hz))
        self.root.after(delay_ms, self._schedule_publish)

    def close(self) -> None:
        self.closed = True
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_z_values(raw_value: str) -> List[float]:
    try:
        return [float(item.strip()) for item in raw_value.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--test-z-values must be a comma-separated float list") from exc


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Piper PosCmd slider publisher")
    parser.add_argument(
        "--test-z-motion",
        action="store_true",
        help="run a headless z-only motion test and exit",
    )
    parser.add_argument(
        "--test-z-values",
        type=parse_z_values,
        default=parse_z_values("0.230,0.250,0.230"),
        help="comma-separated z values for --test-z-motion",
    )
    parser.add_argument(
        "--test-hold-sec",
        type=float,
        default=2.0,
        help="seconds to publish each z test target",
    )
    return parser.parse_args(argv)


def main(args=None) -> None:
    non_ros_args = remove_ros_args(args=args)
    cli_args = parse_args(non_ros_args[1:])

    rclpy.init(args=args)
    node = PiperPosSliderNode()
    try:
        if cli_args.test_z_motion:
            node.run_test_z_motion(cli_args.test_z_values, max(0.1, cli_args.test_hold_sec))
        else:
            gui = PiperPosSliderGui(node)
            gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
