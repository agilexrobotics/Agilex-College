#!/usr/bin/env python3

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import Pose
from piper_msgs.msg import PosCmd
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_euler_xyz(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float]:
    """Convert a quaternion to roll, pitch and yaw in radians."""
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
    return roll, pitch, yaw


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def limit_step(current: float, target: float, max_delta: float) -> float:
    """Move current toward target by at most max_delta."""
    if max_delta <= 0.0:
        return target

    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


def smooth_pose_toward(
    current: Tuple[float, float, float, float, float, float],
    target: Tuple[float, float, float, float, float, float],
    max_linear_step: float,
    max_angular_step: float,
    position_deadband: float = 0.0,
    angle_deadband: float = 0.0,
) -> Tuple[float, float, float, float, float, float]:
    """Limit translation and rotation changes per publish cycle."""
    target_x, target_y, target_z = target[:3]
    if abs(target_x - current[0]) <= position_deadband:
        target_x = current[0]
    if abs(target_y - current[1]) <= position_deadband:
        target_y = current[1]
    if abs(target_z - current[2]) <= position_deadband:
        target_z = current[2]

    x = limit_step(current[0], target_x, max_linear_step)
    y = limit_step(current[1], target_y, max_linear_step)
    z = limit_step(current[2], target_z, max_linear_step)

    roll_delta = normalize_angle(target[3] - current[3])
    pitch_delta = normalize_angle(target[4] - current[4])
    yaw_delta = normalize_angle(target[5] - current[5])
    if abs(roll_delta) <= angle_deadband:
        roll_delta = 0.0
    if abs(pitch_delta) <= angle_deadband:
        pitch_delta = 0.0
    if abs(yaw_delta) <= angle_deadband:
        yaw_delta = 0.0

    roll = normalize_angle(
        current[3] + limit_step(0.0, roll_delta, max_angular_step)
    )
    pitch = normalize_angle(
        current[4] + limit_step(0.0, pitch_delta, max_angular_step)
    )
    yaw = normalize_angle(
        current[5] + limit_step(0.0, yaw_delta, max_angular_step)
    )
    return x, y, z, roll, pitch, yaw


def transform_to_target_values(
    transform,
) -> Tuple[float, float, float, float, float, float]:
    """Extract x/y/z and roll/pitch/yaw from a TF transform."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    roll, pitch, yaw = quaternion_to_euler_xyz(
        rotation.x, rotation.y, rotation.z, rotation.w
    )
    return translation.x, translation.y, translation.z, roll, pitch, yaw


def pose_to_target_values(
    pose: Pose,
) -> Tuple[float, float, float, float, float, float]:
    """Extract x/y/z and roll/pitch/yaw from a Pose message."""
    roll, pitch, yaw = quaternion_to_euler_xyz(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    return pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw


def make_pos_cmd(
    values: Tuple[float, float, float, float, float, float],
    gripper: float,
    mode1: int,
    mode2: int,
) -> PosCmd:
    """Create the Piper command message from x/y/z and roll/pitch/yaw."""
    x, y, z, roll, pitch, yaw = values
    command = PosCmd()
    command.x = x
    command.y = y
    command.z = z
    command.roll = roll
    command.pitch = pitch
    command.yaw = yaw
    command.gripper = gripper
    command.mode1 = mode1
    command.mode2 = mode2
    return command


class PiperTfControlNode(Node):
    """Follow an arm_pose TF with the Piper position command interface."""

    def __init__(self) -> None:
        super().__init__("piper_tf_control")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("target_frame", "arm_pose")
        self.declare_parameter("command_topic", "pos_cmd")
        self.declare_parameter("feedback_topic", "end_pose")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("smoothing_enabled", True)
        self.declare_parameter("max_linear_speed_m_s", 0.03)
        self.declare_parameter("max_angular_speed_rad_s", 0.6)
        self.declare_parameter("position_deadband_m", 0.002)
        self.declare_parameter("angle_deadband_rad", 0.03)
        self.declare_parameter("gripper", 0.0)
        self.declare_parameter("mode1", 0)
        self.declare_parameter("mode2", 0)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.publish_rate_hz = max(
            1.0,
            min(100.0, float(self.get_parameter("publish_rate_hz").value)),
        )
        self.smoothing_enabled = bool(
            self.get_parameter("smoothing_enabled").value
        )
        self.max_linear_speed_m_s = max(
            0.0, float(self.get_parameter("max_linear_speed_m_s").value)
        )
        self.max_angular_speed_rad_s = max(
            0.0, float(self.get_parameter("max_angular_speed_rad_s").value)
        )
        self.position_deadband_m = max(
            0.0, float(self.get_parameter("position_deadband_m").value)
        )
        self.angle_deadband_rad = max(
            0.0, float(self.get_parameter("angle_deadband_rad").value)
        )
        self.gripper = float(self.get_parameter("gripper").value)
        self.mode1 = int(self.get_parameter("mode1").value)
        self.mode2 = int(self.get_parameter("mode2").value)

        self.command_publisher = self.create_publisher(
            PosCmd, self.command_topic, 10
        )
        self.feedback_pose = None
        self.create_subscription(
            Pose, self.feedback_topic, self._feedback_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_tf_warning = ""
        self._holding_without_tf = False
        self._waiting_for_feedback = False
        self._last_command_values = None
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._publish_target
        )

        self.get_logger().info(
            "启动 TF 控制：%s -> %s，完整位姿发布到 %s，%.1f Hz；"
            "TF 缺失时使用 %s 保持原地；平滑=%s，线速度=%.3f m/s，"
            "角速度=%.3f rad/s，位置死区=%.4f m，角度死区=%.4f rad"
            % (
                self.base_frame,
                self.target_frame,
                self.command_topic,
                self.publish_rate_hz,
                self.feedback_topic,
                self.smoothing_enabled,
                self.max_linear_speed_m_s,
                self.max_angular_speed_rad_s,
                self.position_deadband_m,
                self.angle_deadband_rad,
            )
        )

    def _feedback_callback(self, pose: Pose) -> None:
        self.feedback_pose = pose

    def _lookup_target(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                rclpy.time.Time(),
            )
            if self._holding_without_tf:
                self.get_logger().info(
                    "已重新获取 TF %s -> %s，恢复跟随目标"
                    % (self.base_frame, self.target_frame)
                )
            self._holding_without_tf = False
            self._waiting_for_feedback = False
            return transform
        except TransformException as exc:
            warning = f"等待 TF {self.base_frame} -> {self.target_frame}: {exc}"
            if warning != self._last_tf_warning:
                self.get_logger().warn(warning)
                self._last_tf_warning = warning
            return None

    def _publish_target(self) -> None:
        target_transform = self._lookup_target()
        hold_mode = False
        if target_transform is not None:
            values = transform_to_target_values(target_transform)
        elif self.feedback_pose is not None:
            if not self._holding_without_tf:
                self.get_logger().warn(
                    "TF 缺失，使用最新 %s 反馈持续保持原地"
                    % self.feedback_topic
                )
            self._holding_without_tf = True
            self._waiting_for_feedback = False
            values = pose_to_target_values(self.feedback_pose)
            hold_mode = True
        else:
            if not self._waiting_for_feedback:
                self.get_logger().warn(
                    "TF 和 %s 均不可用，暂不发布 PosCmd"
                    % self.feedback_topic
                )
            self._waiting_for_feedback = True
            return

        if self.smoothing_enabled and not hold_mode:
            if self._last_command_values is None:
                if (
                    self.feedback_pose is not None
                    and target_transform is not None
                ):
                    current = pose_to_target_values(self.feedback_pose)
                else:
                    current = values
            else:
                current = self._last_command_values

            values = smooth_pose_toward(
                current,
                values,
                self.max_linear_speed_m_s / self.publish_rate_hz,
                self.max_angular_speed_rad_s / self.publish_rate_hz,
                self.position_deadband_m,
                self.angle_deadband_rad,
            )

        command = make_pos_cmd(values, self.gripper, self.mode1, self.mode2)
        self.command_publisher.publish(command)
        self._last_command_values = values

    def destroy_node(self):
        self.timer.cancel()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiperTfControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
