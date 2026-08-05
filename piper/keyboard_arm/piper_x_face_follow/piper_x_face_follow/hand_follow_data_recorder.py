#!/usr/bin/env python3
"""Crash-tolerant JSONL/CSV recorder for real PIPER-X hand-follow tests."""

import csv
import json
from pathlib import Path
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from piper_msgs.msg import PiperStatusMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool, Float32, String, UInt8
from tf2_msgs.msg import TFMessage


class HandFollowDataRecorder(Node):
    def __init__(self) -> None:
        super().__init__("piper_x_hand_follow_data_recorder")
        self.declare_parameter("output_directory", "")
        self.declare_parameter("duration_sec", 30)
        output = str(self.get_parameter("output_directory").value)
        if not output:
            raise ValueError("output_directory is required")
        self._output = Path(output)
        self._output.mkdir(parents=True, exist_ok=False)
        self._duration = max(
            1.0, float(self.get_parameter("duration_sec").value)
        )
        self._started = time.monotonic()
        self.done = False
        self._counts = {}
        self._event_count = 0
        self._events = (self._output / "events.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )
        self._joint_file = (self._output / "joints.csv").open(
            "w", encoding="utf-8", newline="", buffering=1
        )
        self._joint_csv = csv.writer(self._joint_file)
        self._joint_csv.writerow(
            ["wall_time", "ros_time_ns", "source"]
            + [f"joint{index}_deg" for index in range(1, 6)]
        )

        topic_types = (
            ("/joint_states_single", JointState),
            ("/joint_ctrl_single", JointState),
            ("/piper_x_hand_follow/preview_joint_states", JointState),
            ("/arm_status", PiperStatusMsg),
            ("/drag_teach_active", Bool),
            ("/human_tracking/status", String),
            ("/human_tracking/gesture_command", String),
            ("/human_tracking/visual_hand_target", PointStamped),
            ("/human_tracking/visible_hand_count", UInt8),
            ("/human_tracking/hand_layout", String),
            ("/human_tracking/hand_distance_m", Float32),
            ("/human_tracking/typing_plane", PoseStamped),
            ("/piper_x_hand_follow/status", String),
            ("/camera/color/camera_info", CameraInfo),
            ("/tf", TFMessage),
            ("/tf_static", TFMessage),
        )
        for topic, message_type in topic_types:
            self.create_subscription(
                message_type,
                topic,
                lambda message, name=topic: self._record(name, message),
                qos_profile_sensor_data,
            )
        self.create_timer(0.05, self._check_duration)
        self.get_logger().info(
            f"Recording {len(topic_types)} topics for {self._duration:.1f}s "
            f"to {self._output}"
        )

    def _record(self, topic: str, message) -> None:
        if self.done:
            return
        now = time.time()
        ros_now = self.get_clock().now().nanoseconds
        payload = message_to_ordereddict(message)
        self._events.write(
            json.dumps(
                {
                    "wall_time": now,
                    "ros_time_ns": ros_now,
                    "topic": topic,
                    "message": payload,
                },
                ensure_ascii=False,
                separators=(",", ":"),
            )
            + "\n"
        )
        self._counts[topic] = self._counts.get(topic, 0) + 1
        self._event_count += 1
        if isinstance(message, JointState) and len(message.position) >= 5:
            self._joint_csv.writerow(
                [now, ros_now, topic]
                + [
                    float(position) * 180.0 / 3.141592653589793
                    for position in message.position[:5]
                ]
            )
        if self._event_count % 20 == 0:
            self._events.flush()
            self._joint_file.flush()

    def _check_duration(self) -> None:
        if time.monotonic() - self._started >= self._duration:
            self.done = True

    def close(self, interrupted: bool = False) -> None:
        if self._events.closed:
            return
        self.done = True
        self._events.flush()
        self._joint_file.flush()
        self._events.close()
        self._joint_file.close()
        metadata = {
            "format": "piper_x_hand_follow_jsonl_v1",
            "duration_requested_sec": self._duration,
            "duration_actual_sec": round(time.monotonic() - self._started, 3),
            "interrupted": bool(interrupted),
            "event_count": self._event_count,
            "topic_counts": self._counts,
        }
        (self._output / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def destroy_node(self):
        self.close(interrupted=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandFollowDataRecorder()
    interrupted = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        node.close(interrupted=interrupted)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
