#!/usr/bin/env python3
"""Conservative PIPER-X image-based visual servo with an immutable J6.

本节点默认只做预览，不会直接驱动机械臂。真实控制指令只有在
actual_motion=true、显式调用 enable_tracking 服务、目标和反馈状态都满足
安全门控时，才会发布到 command_topic。
"""

import json
import math
import time
from collections import deque
from typing import Optional, Tuple

from geometry_msgs.msg import PointStamped
import rclpy
from piper_msgs.msg import PiperStatusMsg
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger
from smart_keyboard_msgs.msg import UserState

from .policy import (
    clamp,
    FaceStabilityFilter,
    deadband,
    enforce_j6_lock,
    ordered_joints,
    slew,
)

# 关节安全边界来自 PIPER-X URDF，并给 J2/J3 的机械零位留少量调试余量。
# 后续真实控制指令会先被 clamp 到这些范围内，再进入发布门控。
_SAFE_JOINT_BOUNDS = (
    (-2.568, 2.568),
    (-0.05, 3.09),
    (-2.9171, 0.05),
    (-1.52, 1.52),
    (-1.52, 1.52),
)


class FaceFollowController(Node):
    def __init__(self) -> None:
        super().__init__("piper_x_face_follow_controller")
        # actual_motion 是真实运动总开关。false 时只发布 preview，不发机械臂命令。
        self.declare_parameter("actual_motion", False)
        self.declare_parameter("feedback_topic", "/joint_states_single")
        # command_topic 是真实机械臂控制 topic，默认接 piper_ros 的单臂关节控制入口。
        self.declare_parameter("command_topic", "/joint_ctrl_single")
        self.declare_parameter("face_topic", "/face_landmarks_2d")
        self.declare_parameter("hand_topic", "/human_tracking/hand_midpoint")
        # tracking_source 决定跟踪人脸 2D 中心，还是跟踪 MediaPipe+深度得到的双手 3D 中点。
        self.declare_parameter("tracking_source", "face")
        # base_cartesian 模式依赖相机到机械臂坐标的外参，必须显式声明已标定。
        self.declare_parameter("tf_calibrated", False)
        self.declare_parameter("control_frame_mode", "camera_image_servo")
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("face_timeout_sec", 0.6)
        self.declare_parameter("feedback_timeout_sec", 0.3)
        self.declare_parameter("stable_samples", 8)
        self.declare_parameter("stable_span_px", 18.0)
        self.declare_parameter("image_center_u", 320.0)
        self.declare_parameter("image_center_v", 240.0)
        self.declare_parameter("deadband_u_px", 24.0)
        self.declare_parameter("deadband_v_px", 20.0)
        self.declare_parameter("j1_gain_rad_per_px", -0.0008)
        self.declare_parameter("j2_gain_rad_per_px", 0.00045)
        self.declare_parameter("j3_gain_rad_per_px", -0.00045)
        self.declare_parameter("j1_max_offset_deg", 6.0)
        self.declare_parameter("j23_max_offset_deg", 4.0)
        self.declare_parameter("max_speed_deg_s", 2.0)
        self.declare_parameter("driver_speed_percent", 5)
        self.declare_parameter("j6_drift_limit_deg", 0.5)
        self.declare_parameter("feedback_latch_settle_sec", 0.5)
        self.declare_parameter("feedback_latch_span_deg", 0.5)
        self.declare_parameter("require_nonzero_feedback_stamp", True)
        self.declare_parameter("require_arm_status", True)
        self.declare_parameter("desired_hand_x_m", 0.0)
        self.declare_parameter("desired_hand_y_m", 0.12)
        self.declare_parameter("desired_hand_z_m", 0.55)
        self.declare_parameter("hand_stable_span_m", 0.04)
        self.declare_parameter("j1_gain_rad_per_m", -0.70)
        self.declare_parameter("j2_vertical_gain_rad_per_m", 0.25)
        self.declare_parameter("j3_vertical_gain_rad_per_m", -0.25)
        self.declare_parameter("j2_depth_gain_rad_per_m", 0.0)
        self.declare_parameter("j3_depth_gain_rad_per_m", 0.0)

        self._actual_motion = bool(self.get_parameter("actual_motion").value)
        self._tracking_source = str(self.get_parameter("tracking_source").value).lower()
        if self._tracking_source not in ("face", "hands"):
            raise ValueError("tracking_source must be 'face' or 'hands'")
        self._tf_calibrated = bool(self.get_parameter("tf_calibrated").value)
        self._control_frame_mode = str(
            self.get_parameter("control_frame_mode").value
        ).lower()
        if self._control_frame_mode not in ("camera_image_servo", "base_cartesian"):
            raise ValueError(
                "control_frame_mode must be camera_image_servo or base_cartesian"
            )
        self._rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self._face_timeout = float(self.get_parameter("face_timeout_sec").value)
        self._feedback_timeout = float(self.get_parameter("feedback_timeout_sec").value)
        self._center_u = float(self.get_parameter("image_center_u").value)
        self._center_v = float(self.get_parameter("image_center_v").value)
        self._deadband_u = float(self.get_parameter("deadband_u_px").value)
        self._deadband_v = float(self.get_parameter("deadband_v_px").value)
        self._gain_j1 = float(self.get_parameter("j1_gain_rad_per_px").value)
        self._gain_j2 = float(self.get_parameter("j2_gain_rad_per_px").value)
        self._gain_j3 = float(self.get_parameter("j3_gain_rad_per_px").value)
        self._max_j1 = math.radians(float(self.get_parameter("j1_max_offset_deg").value))
        self._max_j23 = math.radians(float(self.get_parameter("j23_max_offset_deg").value))
        self._max_step = math.radians(
            float(self.get_parameter("max_speed_deg_s").value)
        ) / self._rate
        self._driver_speed_percent = int(
            clamp(float(self.get_parameter("driver_speed_percent").value), 1.0, 20.0)
        )
        self._j6_drift_limit = math.radians(
            float(self.get_parameter("j6_drift_limit_deg").value)
        )
        self._feedback_latch_settle = max(
            0.1, float(self.get_parameter("feedback_latch_settle_sec").value)
        )
        self._feedback_latch_span = math.radians(
            max(0.05, float(self.get_parameter("feedback_latch_span_deg").value))
        )
        self._require_nonzero_feedback_stamp = bool(
            self.get_parameter("require_nonzero_feedback_stamp").value
        )
        self._require_arm_status = bool(self.get_parameter("require_arm_status").value)
        self._desired_hand = (
            float(self.get_parameter("desired_hand_x_m").value),
            float(self.get_parameter("desired_hand_y_m").value),
            float(self.get_parameter("desired_hand_z_m").value),
        )
        self._hand_span_limit = float(self.get_parameter("hand_stable_span_m").value)
        self._hand_gains = (
            float(self.get_parameter("j1_gain_rad_per_m").value),
            float(self.get_parameter("j2_vertical_gain_rad_per_m").value),
            float(self.get_parameter("j3_vertical_gain_rad_per_m").value),
            float(self.get_parameter("j2_depth_gain_rad_per_m").value),
            float(self.get_parameter("j3_depth_gain_rad_per_m").value),
        )
        self._filter = FaceStabilityFilter(
            int(self.get_parameter("stable_samples").value),
            float(self.get_parameter("stable_span_px").value),
        )

        self._feedback: Optional[Tuple[float, ...]] = None
        self._anchor: Optional[Tuple[float, ...]] = None
        self._command: Optional[Tuple[float, ...]] = None
        # J6 在本控制器中是会话级硬锁：启动稳定后锁定，真实命令发布前再次强制写回。
        self._j6_lock: Optional[float] = None
        self._latch_candidate: Optional[Tuple[float, ...]] = None
        self._latch_candidate_since = 0.0
        self._warned_zero_stamp = False
        self._stable_face = None
        self._hand_samples = deque(
            maxlen=max(2, int(self.get_parameter("stable_samples").value))
        )
        self._stable_hand = None
        self._last_hand_time = 0.0
        self._hands_on_keyboard = False
        self._both_hands_3d = False
        self._person_detected = False
        self._last_face_time = 0.0
        self._last_feedback_time = 0.0
        self._tracking_enabled = False
        self._fault = ""
        self._arm_status_seen = False
        self._arm_error = -1

        feedback_topic = str(self.get_parameter("feedback_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        face_topic = str(self.get_parameter("face_topic").value)
        # _command_pub 是唯一真实控制出口；不要把 preview topic 当成机械臂命令。
        self._command_pub = self.create_publisher(JointState, command_topic, 1)
        self._preview_pub = self.create_publisher(
            JointState, "/piper_x_face_follow/preview_joint_states", 10
        )
        self._status_pub = self.create_publisher(
            String, "/piper_x_face_follow/status", 10
        )
        self.create_subscription(JointState, feedback_topic, self._feedback_cb, 10)
        self.create_subscription(Float32MultiArray, face_topic, self._face_cb, 10)
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("hand_topic").value),
            self._hand_cb,
            10,
        )
        self.create_subscription(
            UserState, "/smart_keyboard/user_state", self._user_state_cb, 10
        )
        self.create_subscription(
            Bool, "/human_tracking/both_hands_3d", self._both_hands_cb, 10
        )
        self.create_subscription(PiperStatusMsg, "/arm_status", self._arm_status_cb, 10)
        self.create_service(SetBool, "/piper_x_face_follow/enable_tracking", self._enable_cb)
        self.create_service(Trigger, "/piper_x_face_follow/clear_fault", self._clear_fault_cb)
        self.create_service(Trigger, "/piper_x_face_follow/relatch_reference", self._relatch_cb)
        self.create_timer(1.0 / self._rate, self._control_tick)
        self.create_timer(0.5, self._publish_status)

        mode = "REAL COMMAND GATE AVAILABLE" if self._actual_motion else "PREVIEW ONLY"
        self.get_logger().warn(f"{mode}; tracking starts disabled.")
        self.get_logger().warn("J6 will be latched from the first valid hardware feedback frame.")

    def _feedback_cb(self, msg: JointState) -> None:
        joints = ordered_joints(msg.name, msg.position)
        if joints is None:
            return
        # 启动时可能收到零时间戳的伪 joint state；默认忽略，等待真实 CAN 反馈。
        if (
            self._require_nonzero_feedback_stamp
            and msg.header.stamp.sec == 0
            and msg.header.stamp.nanosec == 0
        ):
            if not self._warned_zero_stamp:
                self.get_logger().warn(
                    "Ignoring zero-timestamp startup joint state; waiting for a real CAN feedback frame."
                )
                self._warned_zero_stamp = True
            return
        now = time.monotonic()
        self._feedback = joints
        self._last_feedback_time = now
        if self._j6_lock is None:
            # 只有反馈在 settle 窗口内稳定，才把当前姿态作为 anchor 并锁定 J6。
            if self._latch_candidate is None:
                self._latch_candidate = joints
                self._latch_candidate_since = now
                return
            if max(
                abs(value - baseline)
                for value, baseline in zip(joints, self._latch_candidate)
            ) > self._feedback_latch_span:
                # CAN joint frames can arrive in groups during startup. Restart
                # the settle window until the complete physical pose is stable.
                self._latch_candidate = joints
                self._latch_candidate_since = now
                return
            if now - self._latch_candidate_since < self._feedback_latch_settle:
                return
            self._j6_lock = joints[5]
            self._anchor = joints
            self._command = joints
            self.get_logger().warn(
                f"J6 HARD LOCK latched at {math.degrees(self._j6_lock):.3f} deg."
            )
        elif abs(joints[5] - self._j6_lock) > self._j6_drift_limit:
            # 如果真实反馈显示 J6 偏离锁定角度，立即进入 fault 并停止真实发布。
            self._set_fault(
                "J6 feedback drift exceeded limit: "
                f"lock={math.degrees(self._j6_lock):.3f} deg, "
                f"feedback={math.degrees(joints[5]):.3f} deg"
            )

    def _face_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 11:
            self._stable_face = None
            self._filter.clear()
            return
        self._last_face_time = time.monotonic()
        # 人脸控制只使用检测框中心点；必须连续稳定若干帧后才允许进入控制计算。
        self._stable_face = self._filter.add(float(msg.data[9]), float(msg.data[10]))

    def _hand_cb(self, msg: PointStamped) -> None:
        point = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        if not all(math.isfinite(value) for value in point):
            self._hand_samples.clear()
            self._stable_hand = None
            return
        self._last_hand_time = time.monotonic()
        self._hand_samples.append(point)
        if len(self._hand_samples) < self._hand_samples.maxlen:
            self._stable_hand = None
            return
        # 手部 3D 点需要在滑动窗口内足够稳定，否则视为目标未就绪。
        spans = [
            max(sample[index] for sample in self._hand_samples)
            - min(sample[index] for sample in self._hand_samples)
            for index in range(3)
        ]
        if max(spans) > self._hand_span_limit:
            self._stable_hand = None
            return
        self._stable_hand = tuple(
            sum(sample[index] for sample in self._hand_samples)
            / len(self._hand_samples)
            for index in range(3)
        )

    def _user_state_cb(self, msg: UserState) -> None:
        self._hands_on_keyboard = bool(msg.hands_on_keyboard)
        self._person_detected = bool(msg.person_detected)

    def _both_hands_cb(self, msg: Bool) -> None:
        self._both_hands_3d = bool(msg.data)

    def _arm_status_cb(self, msg: PiperStatusMsg) -> None:
        self._arm_status_seen = True
        self._arm_error = int(msg.err_code)
        if self._arm_error != 0:
            self._set_fault(f"arm_status err_code={self._arm_error}")

    def _enable_cb(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            self._tracking_enabled = False
            response.success = True
            response.message = "tracking disabled; no further commands will be published"
            return response
        reasons = self._readiness_failures()
        # 即使所有传感器状态都 ready，actual_motion=false 也会拒绝启用真实运动。
        if not self._actual_motion:
            reasons.append("actual_motion=false (preview-only launch)")
        if (
            self._tracking_source == "hands"
            and self._control_frame_mode == "base_cartesian"
            and not self._tf_calibrated
        ):
            # 双手 3D 的 base_cartesian 模式必须先确认相机外参，否则方向可能错误。
            reasons.append("tf_calibrated=false")
        if reasons:
            response.success = False
            response.message = "; ".join(reasons)
            return response
        # enable 成功时把当前关节反馈作为 anchor，后续目标只在 anchor 附近小幅偏移。
        self._anchor = self._feedback
        self._command = self._feedback
        self._tracking_enabled = True
        response.success = True
        response.message = (
            f"tracking enabled; J6 locked at {math.degrees(self._j6_lock):.3f} deg"
        )
        return response

    def _clear_fault_cb(self, _request: Trigger.Request, response: Trigger.Response):
        if self._feedback is None or self._j6_lock is None:
            response.success = False
            response.message = "no joint feedback"
            return response
        if abs(self._feedback[5] - self._j6_lock) > self._j6_drift_limit:
            response.success = False
            response.message = "J6 is still outside the locked-angle tolerance"
            return response
        if self._arm_status_seen and self._arm_error != 0:
            response.success = False
            response.message = f"arm err_code is still {self._arm_error}"
            return response
        self._fault = ""
        response.success = True
        response.message = "fault cleared; tracking remains disabled"
        return response

    def _relatch_cb(self, _request: Trigger.Request, response: Trigger.Response):
        if self._tracking_enabled:
            response.success = False
            response.message = "disable tracking before relatching"
            return response
        if self._feedback is None:
            response.success = False
            response.message = "no joint feedback"
            return response
        # This is the only place, besides startup, where the immutable session lock
        # can change. It requires an explicit service call while motion is disabled.
        self._j6_lock = self._feedback[5]
        self._anchor = self._feedback
        self._command = self._feedback
        self._fault = ""
        response.success = True
        response.message = f"reference relatched; J6={math.degrees(self._j6_lock):.3f} deg"
        return response

    def _readiness_failures(self):
        """返回阻止真实控制发布的原因列表；空列表表示当前安全门控通过。"""
        now = time.monotonic()
        failures = []
        if self._fault:
            failures.append(f"fault={self._fault}")
        if self._feedback is None or self._j6_lock is None:
            failures.append("joint feedback/J6 lock not ready")
        elif now - self._last_feedback_time > self._feedback_timeout:
            failures.append("joint feedback stale")
        if self._tracking_source == "face":
            if self._stable_face is None or now - self._last_face_time > self._face_timeout:
                failures.append("stable face not ready")
        else:
            # 双手模式要求：手部 3D 稳定、检测到人、左右手都有有效深度，且手未压在键盘上。
            if self._stable_hand is None or now - self._last_hand_time > self._face_timeout:
                failures.append("stable 3D hands not ready")
            if not self._person_detected:
                failures.append("person not detected")
            if not self._both_hands_3d:
                failures.append("both hands with valid depth not ready")
            if self._hands_on_keyboard:
                failures.append("hands_on_keyboard: motion hold")
        if self._require_arm_status and not self._arm_status_seen:
            failures.append("arm_status not received")
        elif self._require_arm_status and self._arm_error != 0:
            failures.append(f"arm err_code={self._arm_error}")
        return failures

    def _control_tick(self) -> None:
        # 没有真实关节反馈、anchor 或 J6 锁定值时，连预览命令也不计算。
        if self._feedback is None or self._anchor is None or self._j6_lock is None:
            return
        target_stale = (
            self._stable_face is None
            or time.monotonic() - self._last_face_time > self._face_timeout
            if self._tracking_source == "face"
            else self._stable_hand is None
            or time.monotonic() - self._last_hand_time > self._face_timeout
        )
        if target_stale:
            if self._tracking_enabled:
                self.get_logger().warn("Human target lost/unstable: command stream stopped.")
            self._tracking_enabled = False
            return
        if self._tracking_source == "hands" and self._hands_on_keyboard:
            # 用户手已经压在键盘/托盘区域时保持当前状态，不继续刷新移动目标。
            return
        target = list(self._anchor)
        if self._tracking_source == "face":
            # 人脸模式：按图像中心误差做小范围 J1/J2/J3 视觉伺服。
            error_u = deadband(self._stable_face.u - self._center_u, self._deadband_u)
            error_v = deadband(self._stable_face.v - self._center_v, self._deadband_v)
            target[0] += max(-self._max_j1, min(self._max_j1, self._gain_j1 * error_u))
            target[1] += max(-self._max_j23, min(self._max_j23, self._gain_j2 * error_v))
            target[2] += max(-self._max_j23, min(self._max_j23, self._gain_j3 * error_v))
        else:
            # 双手模式：按 3D 手部中点相对期望位置的误差生成 J1/J2/J3 偏移。
            error_x = self._stable_hand[0] - self._desired_hand[0]
            error_y = self._stable_hand[1] - self._desired_hand[1]
            error_z = self._stable_hand[2] - self._desired_hand[2]
            gain_j1, gain_j2_y, gain_j3_y, gain_j2_z, gain_j3_z = self._hand_gains
            target[0] += clamp(gain_j1 * error_x, -self._max_j1, self._max_j1)
            target[1] += clamp(
                gain_j2_y * error_y + gain_j2_z * error_z,
                -self._max_j23,
                self._max_j23,
            )
            target[2] += clamp(
                gain_j3_y * error_y + gain_j3_z * error_z,
                -self._max_j23,
                self._max_j23,
            )
        for index, bounds in enumerate(_SAFE_JOINT_BOUNDS):
            target[index] = clamp(target[index], bounds[0], bounds[1])
        target[5] = self._j6_lock
        current = self._command if self._command is not None else self._feedback
        # 速度限制用每周期最大步长实现，避免目标跳变直接打到机械臂。
        next_command = slew(current, target, self._max_step)
        self._command = enforce_j6_lock(next_command, self._j6_lock)
        preview = self._make_joint_message(self._command)
        # 预览始终发布，方便 RViz/调试观察；它不代表机械臂一定会运动。
        self._preview_pub.publish(preview)
        if self._tracking_enabled and self._actual_motion:
            # 真实发布前再跑一次 readiness 检查，防止 enable 后状态变坏仍继续运动。
            failures = self._readiness_failures()
            if failures:
                self._tracking_enabled = False
                self.get_logger().error("Command stopped: " + "; ".join(failures))
                return
            guarded = enforce_j6_lock(self._command, self._j6_lock)
            if guarded[5] != self._j6_lock:
                self._set_fault("J6 outlet guard rejected command")
                return
            # 只有这一行会向 command_topic 发布真实机械臂控制指令。
            self._command_pub.publish(self._make_joint_message(guarded, real_command=True))

    def _make_joint_message(
        self, joints: Tuple[float, ...], real_command: bool = False
    ) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "piper_x_face_follow"
        msg.name = [f"joint{i}" for i in range(1, 7)]
        msg.position = list(joints)
        # piper_ros 当前把 velocity[6] 解释为全局速度百分比；真实命令必须显式给第 7 项。
        # 预览消息只给 6 个 0，避免被误当作真实驱动速度配置。
        msg.velocity = (
            [0.0] * 6 + [float(self._driver_speed_percent)]
            if real_command
            else [0.0] * 6
        )
        msg.effort = [0.0] * 6
        return msg

    def _set_fault(self, reason: str) -> None:
        if not self._fault:
            self.get_logger().error("SAFETY FAULT: " + reason)
        self._fault = reason
        # 任何 safety fault 都会关闭 tracking，直到 clear_fault 成功后再手动 enable。
        self._tracking_enabled = False

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "mode": "real-capable" if self._actual_motion else "preview",
                "tracking_source": self._tracking_source,
                "control_frame_mode": self._control_frame_mode,
                "tracking_enabled": self._tracking_enabled,
                "feedback_ready": self._feedback is not None,
                "stable_face": self._stable_face is not None,
                "stable_hands_3d": self._stable_hand is not None,
                "hands_on_keyboard": self._hands_on_keyboard,
                "both_hands_3d": self._both_hands_3d,
                "tf_calibrated": self._tf_calibrated,
                "arm_status_seen": self._arm_status_seen,
                "arm_error": self._arm_error,
                "j6_lock_deg": (
                    round(math.degrees(self._j6_lock), 4) if self._j6_lock is not None else None
                ),
                "j6_latch_pending": self._j6_lock is None,
                "fault": self._fault,
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
