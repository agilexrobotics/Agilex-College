#!/usr/bin/env python3

import math
import time
from typing import Sequence, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _vector3_from_parameter(value, default: Vector3) -> Vector3:
    try:
        if len(value) >= 3:
            return float(value[0]), float(value[1]), float(value[2])
    except Exception:
        pass
    return default


def normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    x, y, z, w = (
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def rotate_vector_by_quaternion(
    vector: Sequence[float], quaternion: Sequence[float]
) -> Vector3:
    """Rotate a vector by a quaternion in xyzw order."""
    vx, vy, vz = float(vector[0]), float(vector[1]), float(vector[2])
    qx, qy, qz, qw = normalize_quaternion(quaternion)

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    rx = vx + qw * tx + (qy * tz - qz * ty)
    ry = vy + qw * ty + (qz * tx - qx * tz)
    rz = vz + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def euler_xyz_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll, pitch and yaw in radians to a quaternion in xyzw order."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return normalize_quaternion((x, y, z, w))


def compute_arm_pose(
    face_translation: Sequence[float],
    face_quaternion: Sequence[float],
    offset_xyz: Sequence[float],
    target_rpy: Sequence[float],
) -> Tuple[Vector3, Quaternion]:
    """Compute base-frame arm_pose translation and fixed configured orientation."""
    del face_quaternion
    target_translation = (
        float(face_translation[0]) + float(offset_xyz[0]),
        float(face_translation[1]) + float(offset_xyz[1]),
        float(face_translation[2]) + float(offset_xyz[2]),
    )
    target_quaternion = euler_xyz_to_quaternion(
        float(target_rpy[0]), float(target_rpy[1]), float(target_rpy[2])
    )
    return target_translation, target_quaternion


def compute_arm_pose_from_transform(
    face_transform: TransformStamped,
    offset_xyz: Sequence[float],
    target_rpy: Sequence[float],
) -> Tuple[Vector3, Quaternion]:
    translation = face_transform.transform.translation
    rotation = face_transform.transform.rotation
    return compute_arm_pose(
        (translation.x, translation.y, translation.z),
        (rotation.x, rotation.y, rotation.z, rotation.w),
        offset_xyz,
        target_rpy,
    )


def make_arm_pose_transform(
    face_transform: TransformStamped,
    target_frame: str,
    offset_xyz: Sequence[float],
    target_rpy: Sequence[float],
) -> TransformStamped:
    target_translation, target_quaternion = compute_arm_pose_from_transform(
        face_transform, offset_xyz, target_rpy
    )

    transform = TransformStamped()
    transform.header.stamp = face_transform.header.stamp
    transform.header.frame_id = face_transform.header.frame_id
    transform.child_frame_id = target_frame
    transform.transform.translation.x = target_translation[0]
    transform.transform.translation.y = target_translation[1]
    transform.transform.translation.z = target_translation[2]
    transform.transform.rotation.x = target_quaternion[0]
    transform.transform.rotation.y = target_quaternion[1]
    transform.transform.rotation.z = target_quaternion[2]
    transform.transform.rotation.w = target_quaternion[3]
    return transform


class FaceTf2PiperLink6Node(Node):
    """Publish base_link -> arm_pose from base_link -> face."""

    def __init__(self) -> None:
        super().__init__("face_tf_2_piper_link6")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("face_frame", "face")
        self.declare_parameter("target_frame", "arm_pose")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("offset_xyz", [-0.189, -0.044, -0.478])
        self.declare_parameter("target_rpy", [0.306, 1.403, 0.280])
        self.declare_parameter("calibrate_from_link6", True)
        self.declare_parameter("calibration_frame", "link6")
        self.declare_parameter("calibration_sample_count", 40)
        self.declare_parameter("calibration_timeout_sec", 8.0)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.face_frame = str(self.get_parameter("face_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.publish_rate_hz = max(
            1.0,
            min(100.0, float(self.get_parameter("publish_rate_hz").value)),
        )
        self.offset_xyz = _vector3_from_parameter(
            self.get_parameter("offset_xyz").value, (-0.189, -0.044, -0.478)
        )
        self.target_rpy = _vector3_from_parameter(
            self.get_parameter("target_rpy").value, (0.306, 1.403, 0.280)
        )
        self.calibrate_from_link6 = bool(
            self.get_parameter("calibrate_from_link6").value
        )
        self.calibration_frame = str(self.get_parameter("calibration_frame").value)
        self.calibration_sample_count = max(
            1, int(self.get_parameter("calibration_sample_count").value)
        )
        self.calibration_timeout_sec = max(
            0.1, float(self.get_parameter("calibration_timeout_sec").value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._last_tf_warning = ""
        self._calibration_samples = []
        self._calibration_started_at = time.monotonic()
        self._calibration_complete = not self.calibrate_from_link6
        self._calibration_failed = False

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._publish_arm_pose
        )

        self.get_logger().info(
            "启动 face 到 Piper 末端目标 TF：%s -> %s 生成 %s；"
            "offset_xyz(base fallback)=%.6f %.6f %.6f；"
            "target_rpy=%.6f %.6f %.6f；%.1f Hz"
            % (
                self.base_frame,
                self.face_frame,
                self.target_frame,
                self.offset_xyz[0],
                self.offset_xyz[1],
                self.offset_xyz[2],
                self.target_rpy[0],
                self.target_rpy[1],
                self.target_rpy[2],
                self.publish_rate_hz,
            )
        )
        if self.calibrate_from_link6:
            self.get_logger().info(
                "将用当前 %s -> %s 和 %s -> %s 自动标定 base 坐标偏移，"
                "采样 %d 次后再发布 %s"
                % (
                    self.base_frame,
                    self.face_frame,
                    self.base_frame,
                    self.calibration_frame,
                    self.calibration_sample_count,
                    self.target_frame,
                )
            )

    def _lookup_face(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame,
                self.face_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            warning = f"等待 TF {self.base_frame} -> {self.face_frame}: {exc}"
            if warning != self._last_tf_warning:
                self.get_logger().warn(warning)
                self._last_tf_warning = warning
            return None

    def _lookup_transform(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            warning = f"等待 TF {target_frame} -> {source_frame}: {exc}"
            if warning != self._last_tf_warning:
                self.get_logger().warn(warning)
                self._last_tf_warning = warning
            return None

    @staticmethod
    def _mean_vector(samples) -> Vector3:
        count = float(len(samples))
        return (
            sum(sample[0] for sample in samples) / count,
            sum(sample[1] for sample in samples) / count,
            sum(sample[2] for sample in samples) / count,
        )

    def _calibrate_offset(self) -> bool:
        if self._calibration_complete:
            return True

        face_transform = self._lookup_transform(self.base_frame, self.face_frame)
        link6_transform = self._lookup_transform(
            self.base_frame, self.calibration_frame
        )
        if face_transform is not None and link6_transform is not None:
            face_translation = face_transform.transform.translation
            link6_translation = link6_transform.transform.translation
            self._calibration_samples.append(
                (
                    link6_translation.x - face_translation.x,
                    link6_translation.y - face_translation.y,
                    link6_translation.z - face_translation.z,
                )
            )

        if len(self._calibration_samples) >= self.calibration_sample_count:
            self.offset_xyz = self._mean_vector(self._calibration_samples)
            self._calibration_complete = True
            self.get_logger().info(
                "自动标定完成：offset_xyz(base)=%.6f %.6f %.6f，samples=%d"
                % (
                    self.offset_xyz[0],
                    self.offset_xyz[1],
                    self.offset_xyz[2],
                    len(self._calibration_samples),
                )
            )
            return True

        elapsed = time.monotonic() - self._calibration_started_at
        if elapsed >= self.calibration_timeout_sec:
            self._calibration_complete = True
            self._calibration_failed = True
            self.get_logger().warn(
                (
                    "自动标定超时，仅收到 %d/%d 个样本；"
                    "使用配置 offset_xyz(base) %.6f %.6f %.6f"
                )
                % (
                    len(self._calibration_samples),
                    self.calibration_sample_count,
                    self.offset_xyz[0],
                    self.offset_xyz[1],
                    self.offset_xyz[2],
                )
            )
            return True

        return False

    def _publish_arm_pose(self) -> None:
        if not self._calibrate_offset():
            return

        face_transform = self._lookup_face()
        if face_transform is None:
            return

        arm_pose_transform = make_arm_pose_transform(
            face_transform, self.target_frame, self.offset_xyz, self.target_rpy
        )
        arm_pose_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(arm_pose_transform)

    def destroy_node(self):
        self.timer.cancel()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceTf2PiperLink6Node()
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
