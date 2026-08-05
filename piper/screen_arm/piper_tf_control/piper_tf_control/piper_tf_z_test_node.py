#!/usr/bin/env python3

import argparse
import sys
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import TransformBroadcaster


class PiperTfZTestNode(Node):
    """Publish a dynamic arm_pose TF that changes z only."""

    def __init__(
        self,
        base_frame: str,
        target_frame: str,
        feedback_topic: str,
        publish_rate_hz: float,
    ) -> None:
        super().__init__("piper_tf_z_test")
        self.base_frame = base_frame
        self.target_frame = target_frame
        self.feedback_topic = feedback_topic
        self.publish_rate_hz = max(1.0, min(100.0, publish_rate_hz))
        self.feedback_pose = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Pose, self.feedback_topic, self._feedback_callback, 10
        )

    def _feedback_callback(self, pose: Pose) -> None:
        self.feedback_pose = pose

    def wait_for_feedback(self, timeout_sec: float) -> Pose:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self.feedback_pose is None:
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "timeout waiting for feedback topic %s"
                    % self.feedback_topic
                )
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.feedback_pose is None:
            raise RuntimeError("feedback pose is unavailable")
        return self.feedback_pose

    def publish_pose(self, base_pose: Pose, target_z: float) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = base_pose.position.x
        transform.transform.translation.y = base_pose.position.y
        transform.transform.translation.z = target_z
        transform.transform.rotation = base_pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def run_sequence(
        self,
        sequence: List[Tuple[float, float]],
        feedback_timeout_sec: float,
    ) -> None:
        base_pose = self.wait_for_feedback(feedback_timeout_sec)
        self.get_logger().info(
            "使用当前 %s 作为测试基准：x=%.6f y=%.6f z=%.6f"
            % (
                self.feedback_topic,
                base_pose.position.x,
                base_pose.position.y,
                base_pose.position.z,
            )
        )

        period = 1.0 / self.publish_rate_hz
        for target_z, hold_sec in sequence:
            self.get_logger().info(
                "发布 %s -> %s: z=%.6f，保持 %.1f 秒"
                % (self.base_frame, self.target_frame, target_z, hold_sec)
            )
            deadline = time.monotonic() + max(0.0, hold_sec)
            while rclpy.ok() and time.monotonic() < deadline:
                self.publish_pose(base_pose, target_z)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)


def parse_sequence(raw_value: str) -> List[Tuple[float, float]]:
    sequence = []
    try:
        for item in raw_value.split(","):
            if not item.strip():
                continue
            z_text, hold_text = item.split(":")
            sequence.append((float(z_text), float(hold_text)))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "sequence must be comma-separated z:seconds pairs"
        ) from exc

    if not sequence:
        raise argparse.ArgumentTypeError("sequence must not be empty")
    return sequence


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a z-only dynamic arm_pose TF for Piper testing"
    )
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--target-frame", default="arm_pose")
    parser.add_argument("--feedback-topic", default="end_pose")
    parser.add_argument("--publish-rate-hz", type=float, default=30.0)
    parser.add_argument("--feedback-timeout-sec", type=float, default=5.0)
    parser.add_argument(
        "--sequence",
        type=parse_sequence,
        default=parse_sequence("0.40:10.0,0.21:10.0"),
        help="comma-separated z:seconds pairs, for example 0.40:10,0.21:10",
    )
    return parser.parse_args(argv)


def main(args=None) -> None:
    non_ros_args = remove_ros_args(args=args)
    cli_args = parse_args(non_ros_args[1:])

    rclpy.init(args=args)
    node = PiperTfZTestNode(
        cli_args.base_frame,
        cli_args.target_frame,
        cli_args.feedback_topic,
        cli_args.publish_rate_hz,
    )
    try:
        node.run_sequence(cli_args.sequence, cli_args.feedback_timeout_sec)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
