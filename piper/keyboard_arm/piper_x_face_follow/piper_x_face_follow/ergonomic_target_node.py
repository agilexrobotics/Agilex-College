#!/usr/bin/env python3
"""Generate a stable keyboard target preview from body and hand geometry."""

from collections import deque
import json
import math
import time

from geometry_msgs.msg import PointStamped, PoseStamped
import rclpy
from rclpy.node import Node
from smart_keyboard_msgs.msg import UserState
from std_msgs.msg import Bool, String


class ErgonomicTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("ergonomic_target")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("stable_samples", 10)
        self.declare_parameter("stable_span_m", 0.04)
        self.declare_parameter("target_timeout_sec", 0.8)
        self.declare_parameter("keyboard_below_hands_m", 0.075)
        self.declare_parameter("keyboard_toward_user_m", 0.02)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._span_limit = float(self.get_parameter("stable_span_m").value)
        self._timeout = float(self.get_parameter("target_timeout_sec").value)
        self._below = float(self.get_parameter("keyboard_below_hands_m").value)
        self._toward = float(self.get_parameter("keyboard_toward_user_m").value)
        sample_count = max(3, int(self.get_parameter("stable_samples").value))
        self._midpoints = deque(maxlen=sample_count)
        self._left = None
        self._right = None
        self._user = None
        self._both_hands = False
        self._last_midpoint_time = 0.0
        self._last_status_signature = None
        self._target_pub = self.create_publisher(
            PoseStamped, "/smart_keyboard/ergonomic_target_preview", 10
        )
        self._status_pub = self.create_publisher(
            String, "/smart_keyboard/ergonomic_target_status", 10
        )
        self.create_subscription(
            PointStamped,
            "/human_tracking/hand_midpoint_base",
            self._midpoint_cb,
            10,
        )
        self.create_subscription(
            PointStamped, "/human_tracking/left_hand_base", self._left_cb, 10
        )
        self.create_subscription(
            PointStamped, "/human_tracking/right_hand_base", self._right_cb, 10
        )
        self.create_subscription(
            UserState, "/smart_keyboard/user_state", self._user_cb, 10
        )
        self.create_subscription(
            Bool, "/human_tracking/both_hands_3d", self._both_hands_cb, 10
        )
        self.create_timer(0.1, self._tick)
        self.get_logger().info("Ergonomic target preview ready.")

    def _midpoint_cb(self, msg: PointStamped) -> None:
        if msg.header.frame_id != self._base_frame:
            return
        point = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        if all(math.isfinite(value) for value in point):
            self._midpoints.append(point)
            self._last_midpoint_time = time.monotonic()

    def _left_cb(self, msg: PointStamped) -> None:
        if msg.header.frame_id == self._base_frame:
            self._left = (msg.point.x, msg.point.y, msg.point.z)

    def _right_cb(self, msg: PointStamped) -> None:
        if msg.header.frame_id == self._base_frame:
            self._right = (msg.point.x, msg.point.y, msg.point.z)

    def _user_cb(self, msg: UserState) -> None:
        self._user = msg

    def _both_hands_cb(self, msg: Bool) -> None:
        self._both_hands = bool(msg.data)

    def _tick(self) -> None:
        reason = self._not_ready_reason()
        if reason:
            self._status(reason, False)
            return
        mean = tuple(
            sum(point[index] for point in self._midpoints) / len(self._midpoints)
            for index in range(3)
        )
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self._base_frame
        # This is the desired keyboard/tray centre, not the link6 command pose.
        # A measured tray TCP transform must be applied before any Cartesian IK.
        target.pose.position.x = mean[0] - self._toward
        target.pose.position.y = mean[1]
        target.pose.position.z = mean[2] - self._below
        yaw = 0.0
        if self._left is not None and self._right is not None:
            yaw = math.atan2(
                float(self._right[1] - self._left[1]),
                float(self._right[0] - self._left[0]),
            )
        target.pose.orientation.z = math.sin(yaw * 0.5)
        target.pose.orientation.w = math.cos(yaw * 0.5)
        self._target_pub.publish(target)
        self._status("stable_target", True)

    def _not_ready_reason(self) -> str:
        if self._user is None or not self._user.person_detected:
            return "person_absent"
        if self._user.hands_on_keyboard:
            return "hands_on_keyboard_hold"
        if not self._both_hands:
            return "both_hands_3d_required"
        if len(self._midpoints) < self._midpoints.maxlen:
            return "collecting_stable_hands"
        if time.monotonic() - self._last_midpoint_time > self._timeout:
            return "hand_target_stale"
        for index in range(3):
            values = [point[index] for point in self._midpoints]
            if max(values) - min(values) > self._span_limit:
                return "hands_moving"
        return ""

    def _status(self, reason: str, ready: bool) -> None:
        signature = (reason, ready)
        if signature != self._last_status_signature:
            self._last_status_signature = signature
            self.get_logger().info(
                f"ERGONOMIC TARGET: ready={ready}, reason={reason}"
            )
        msg = String()
        msg.data = json.dumps(
            {
                "ready": ready,
                "reason": reason,
                "pose": self._user.pose if self._user is not None else "unknown",
                "hands_on_keyboard": (
                    bool(self._user.hands_on_keyboard)
                    if self._user is not None
                    else False
                ),
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ErgonomicTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
