#!/usr/bin/env python3
"""Bounded PIPER-X Cartesian hand servo using J1-J5 while hard-locking J6."""

import json
import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from piper_msgs.msg import PiperStatusMsg
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String, UInt8
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from .kinematics import SerialChain
from .kalman import ConstantVelocityKalman3D
from .depth_tracker import ConfirmedAlphaBeta1D
from .policy import clamp, enforce_j6_lock, ordered_joints
from .servo_math import (
    bounded_single_hand_j5_step,
    demonstrated_j5_layout_step,
    enforce_base_downward_trend,
    physical_j4_vertical_search_step,
    position_level_ik_step,
    visual_search_command,
    workspace_limited_step,
)


_SAFE_BOUNDS = (
    (-2.568, 2.568),
    (-0.05, 3.09),
    (-2.9171, 0.05),
    (-1.52, 1.52),
    (-1.52, 1.52),
)


def _quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def hold_target_changed(
    reference: np.ndarray,
    current: np.ndarray,
    xy_threshold_m: float,
    depth_threshold_m: float,
) -> bool:
    """Return true only for a deliberate, large hand-target displacement."""
    reference = np.asarray(reference, dtype=float)
    current = np.asarray(current, dtype=float)
    if reference.shape != (3,) or current.shape != (3,):
        return False
    if not np.all(np.isfinite(reference)) or not np.all(np.isfinite(current)):
        return False
    return bool(
        np.linalg.norm(current[:2] - reference[:2]) >= xy_threshold_m
        or abs(float(current[2] - reference[2])) >= depth_threshold_m
    )


def hand_loss_home_due(
    target_was_acquired: bool,
    zero_hands_since: float,
    now: float,
    timeout_sec: float,
) -> bool:
    """Return true only after an acquired target has been absent continuously."""
    return bool(
        target_was_acquired
        and zero_hands_since > 0.0
        and timeout_sec > 0.0
        and now - zero_hands_since >= timeout_sec
    )


class CartesianHandController(Node):
    def __init__(self) -> None:
        super().__init__("piper_x_cartesian_hand_controller")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("actual_motion", False)
        self.declare_parameter("mount_confirmed", False)
        self.declare_parameter("servo_mode", "ergonomic_base")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("target_timeout_sec", 0.8)
        self.declare_parameter("feedback_timeout_sec", 0.3)
        self.declare_parameter("j6_drift_limit_deg", 0.5)
        self.declare_parameter("feedback_latch_settle_sec", 0.5)
        self.declare_parameter("feedback_latch_span_deg", 0.5)
        self.declare_parameter("max_joint_speed_deg_s", 0.5)
        self.declare_parameter("j5_max_joint_speed_deg_s", 4.0)
        self.declare_parameter("j5_max_joint_accel_deg_s2", 40.0)
        self.declare_parameter("max_joint_accel_deg_s2", 10.0)
        self.declare_parameter("max_joint_offset_deg", 2.0)
        for index in range(1, 6):
            self.declare_parameter(f"max_joint{index}_offset_deg", -1.0)
        self.declare_parameter("max_cartesian_step_m", 0.003)
        self.declare_parameter("max_target_delta_m", 0.08)
        self.declare_parameter("minimum_camera_radial_m", 0.18)
        self.declare_parameter("below_base_minimum_radial_m", 0.18)
        self.declare_parameter("minimum_camera_z_m", -0.35)
        self.declare_parameter("maximum_camera_z_m", 0.75)
        self.declare_parameter("orientation_weight_m_per_rad", 0.15)
        self.declare_parameter("damping", 0.04)
        self.declare_parameter("driver_speed_percent", 2)
        self.declare_parameter("auto_start_tracking", False)
        self.declare_parameter("require_start_gesture", False)
        self.declare_parameter("home_tolerance_deg", 0.6)
        self.declare_parameter("home_return_speed_scale", 1.0)
        self.declare_parameter("settle_hold_enabled", False)
        self.declare_parameter("settle_center_error_deg", 8.0)
        self.declare_parameter("settle_hold_time_sec", 0.7)
        self.declare_parameter("settle_dropout_grace_sec", 0.5)
        self.declare_parameter("settle_orientation_error_deg", 6.0)
        self.declare_parameter("hold_reacquire_xy_m", 0.10)
        self.declare_parameter("hold_reacquire_depth_m", 0.10)
        self.declare_parameter("hold_reacquire_time_sec", 0.5)
        self.declare_parameter("hand_loss_home_sec", 0.0)
        self.declare_parameter("startup_nudge_deg", 0.0)
        self.declare_parameter("startup_nudge_timeout_sec", 15.0)
        self.declare_parameter("tcp_x", 0.0)
        self.declare_parameter("tcp_y", 0.0)
        self.declare_parameter("tcp_z", 0.0)
        self.declare_parameter("desired_hand_x_m", 0.0)
        self.declare_parameter("desired_hand_y_m", 0.0)
        self.declare_parameter("desired_hand_z_m", 0.50)
        self.declare_parameter("minimum_hand_distance_m", 0.25)
        self.declare_parameter("maximum_hand_distance_m", 0.35)
        self.declare_parameter("hand_xy_deadband_m", 0.01)
        self.declare_parameter("hand_z_deadband_m", 0.02)
        self.declare_parameter("kalman_process_noise", 1.0)
        self.declare_parameter("kalman_measurement_noise", 0.0025)
        self.declare_parameter("kalman_innovation_gate_m", 0.20)
        self.declare_parameter("kalman_max_velocity_m_s", 1.5)
        self.declare_parameter("prediction_lead_sec", 0.06)
        self.declare_parameter("kalman_min_updates", 3)
        self.declare_parameter("depth_timeout_sec", 0.55)
        self.declare_parameter("depth_jump_gate_m", 0.12)
        self.declare_parameter("depth_stable_updates", 3)
        self.declare_parameter("distance_reacquire_margin_m", 0.04)
        self.declare_parameter("align_xy_gain", 0.65)
        self.declare_parameter("typing_plane_timeout_sec", 0.55)
        self.declare_parameter("typing_plane_stable_updates", 4)
        self.declare_parameter("typing_plane_filter_alpha", 0.22)
        self.declare_parameter("keyboard_max_tilt_deg", 25.0)
        self.declare_parameter("orientation_error_limit_deg", 30.0)
        self.declare_parameter("visual_target_timeout_sec", 0.45)
        self.declare_parameter("visual_filter_alpha", 0.45)
        self.declare_parameter("tracking_center_vertical_offset_deg", 0.0)
        self.declare_parameter("tracking_vertical_translation_sign", 1.0)
        self.declare_parameter("occlusion_downward_step_m", 0.0)
        self.declare_parameter("tracking_horizontal_gain", 1.0)
        self.declare_parameter("single_hand_horizontal_gain", 1.0)
        self.declare_parameter("visual_center_deadband_deg", 4.0)
        self.declare_parameter("visual_center_stable_updates", 3)
        self.declare_parameter("search_prediction_timeout_sec", 0.80)
        self.declare_parameter("single_hand_retreat_m", 0.035)
        self.declare_parameter("j4_image_gain", 0.25)
        self.declare_parameter("j5_image_gain", 0.45)
        self.declare_parameter("j4_image_sign", 1.0)
        self.declare_parameter("j5_image_sign", -1.0)
        self.declare_parameter("j4_low_head_allowance_deg", 6.0)
        self.declare_parameter("wrist_level_gain", 0.80)
        self.declare_parameter("wrist_level_error_limit_deg", 20.0)
        self.declare_parameter("hand_layout_timeout_sec", 0.45)
        self.declare_parameter("j5_layout_gain", 0.20)
        self.declare_parameter("j5_layout_sign", 1.0)
        self.declare_parameter("j5_layout_deadband_deg", 2.5)
        self.declare_parameter("j5_layout_step_limit_deg", 12.0)
        self.declare_parameter("j5_camera_right_stop_deg", 52.0)
        self.declare_parameter("j5_search_right_limit_deg", 60.0)
        self.declare_parameter("j5_stop_tolerance_deg", 0.5)
        self.declare_parameter("single_hand_j5_search_sign", 1.0)
        self.declare_parameter("single_hand_j5_search_step_deg", 6.0)
        self.declare_parameter("single_hand_j5_search_limit_deg", 20.0)
        self.declare_parameter("single_hand_j5_reverse_limit_deg", 10.0)
        self.declare_parameter("single_hand_j5_start_delay_sec", 0.35)
        self.declare_parameter("use_demonstrated_orientation", False)
        self.declare_parameter("demonstrated_joint1_deg", 48.5)
        self.declare_parameter("demonstrated_joint2_deg", 158.0)
        self.declare_parameter("demonstrated_joint3_deg", -56.2)
        self.declare_parameter("demonstrated_joint4_deg", -83.7)
        self.declare_parameter("demonstrated_joint5_deg", 48.8)
        self.declare_parameter("nullspace_posture_gain", 0.05)

        description = str(self.get_parameter("robot_description").value)
        if not description:
            raise ValueError("robot_description is required")
        self._chain = SerialChain.from_urdf(description)
        self._actual_motion = bool(self.get_parameter("actual_motion").value)
        self._mount_confirmed = bool(self.get_parameter("mount_confirmed").value)
        self._servo_mode = str(self.get_parameter("servo_mode").value)
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        if self._servo_mode not in ("ergonomic_base", "eye_in_hand"):
            raise ValueError("servo_mode must be ergonomic_base or eye_in_hand")
        self._rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self._target_timeout = float(self.get_parameter("target_timeout_sec").value)
        self._feedback_timeout = float(self.get_parameter("feedback_timeout_sec").value)
        self._j6_limit = math.radians(float(self.get_parameter("j6_drift_limit_deg").value))
        self._latch_settle = float(self.get_parameter("feedback_latch_settle_sec").value)
        self._latch_span = math.radians(float(self.get_parameter("feedback_latch_span_deg").value))
        self._max_joint_velocity = math.radians(
            float(self.get_parameter("max_joint_speed_deg_s").value)
        )
        self._j5_max_joint_velocity = math.radians(
            float(self.get_parameter("j5_max_joint_speed_deg_s").value)
        )
        self._j5_max_joint_acceleration = math.radians(
            float(self.get_parameter("j5_max_joint_accel_deg_s2").value)
        )
        self._max_joint_acceleration = math.radians(
            float(self.get_parameter("max_joint_accel_deg_s2").value)
        )
        self._max_offset = math.radians(
            float(self.get_parameter("max_joint_offset_deg").value)
        )
        self._max_offsets = np.asarray(
            [
                math.radians(
                    value
                    if value > 0.0
                    else math.degrees(self._max_offset)
                )
                for value in (
                    float(
                        self.get_parameter(f"max_joint{index}_offset_deg").value
                    )
                    for index in range(1, 6)
                )
            ]
        )
        self._max_cart_step = float(self.get_parameter("max_cartesian_step_m").value)
        self._max_target_delta = float(self.get_parameter("max_target_delta_m").value)
        self._workspace = (
            float(self.get_parameter("minimum_camera_radial_m").value),
            float(self.get_parameter("minimum_camera_z_m").value),
            float(self.get_parameter("maximum_camera_z_m").value),
            float(
                self.get_parameter("below_base_minimum_radial_m").value
            ),
        )
        if self._workspace[1] >= self._workspace[2]:
            raise ValueError("minimum camera z must be below maximum camera z")
        self._orientation_weight = float(
            self.get_parameter("orientation_weight_m_per_rad").value
        )
        self._damping = float(self.get_parameter("damping").value)
        self._speed_percent = int(
            clamp(float(self.get_parameter("driver_speed_percent").value), 1.0, 100.0)
        )
        self._auto_start = bool(self.get_parameter("auto_start_tracking").value)
        self._require_start_gesture = bool(
            self.get_parameter("require_start_gesture").value
        )
        self._home_tolerance = math.radians(
            max(0.1, float(self.get_parameter("home_tolerance_deg").value))
        )
        self._home_return_speed_scale = clamp(
            float(self.get_parameter("home_return_speed_scale").value),
            0.1,
            1.0,
        )
        self._settle_hold_enabled = bool(
            self.get_parameter("settle_hold_enabled").value
        )
        self._settle_center_error = math.radians(
            max(
                1.0,
                float(
                    self.get_parameter("settle_center_error_deg").value
                ),
            )
        )
        self._settle_hold_time = max(
            0.2, float(self.get_parameter("settle_hold_time_sec").value)
        )
        self._settle_dropout_grace = max(
            0.0, float(self.get_parameter("settle_dropout_grace_sec").value)
        )
        self._settle_orientation_error = math.radians(
            clamp(
                float(
                    self.get_parameter(
                        "settle_orientation_error_deg"
                    ).value
                ),
                1.0,
                20.0,
            )
        )
        self._hold_reacquire_xy = max(
            0.02, float(self.get_parameter("hold_reacquire_xy_m").value)
        )
        self._hold_reacquire_depth = max(
            0.02, float(self.get_parameter("hold_reacquire_depth_m").value)
        )
        self._hold_reacquire_time = max(
            0.1, float(self.get_parameter("hold_reacquire_time_sec").value)
        )
        self._hand_loss_home_sec = max(
            0.0, float(self.get_parameter("hand_loss_home_sec").value)
        )
        self._nudge_angle = math.radians(
            max(0.0, float(self.get_parameter("startup_nudge_deg").value))
        )
        self._nudge_timeout = float(
            self.get_parameter("startup_nudge_timeout_sec").value
        )
        self._tcp = np.asarray(
            [
                float(self.get_parameter("tcp_x").value),
                float(self.get_parameter("tcp_y").value),
                float(self.get_parameter("tcp_z").value),
            ]
        )
        self._desired_hand = np.asarray(
            [
                float(self.get_parameter("desired_hand_x_m").value),
                float(self.get_parameter("desired_hand_y_m").value),
                float(self.get_parameter("desired_hand_z_m").value),
            ]
        )
        self._hand_distance_range = (
            float(self.get_parameter("minimum_hand_distance_m").value),
            float(self.get_parameter("maximum_hand_distance_m").value),
        )
        if self._hand_distance_range[0] >= self._hand_distance_range[1]:
            raise ValueError("minimum hand distance must be below maximum")
        self._nullspace_gain = float(
            self.get_parameter("nullspace_posture_gain").value
        )
        self._hand_deadband = np.asarray(
            [
                float(self.get_parameter("hand_xy_deadband_m").value),
                float(self.get_parameter("hand_xy_deadband_m").value),
                float(self.get_parameter("hand_z_deadband_m").value),
            ]
        )
        self._kalman_min_updates = max(
            1, int(self.get_parameter("kalman_min_updates").value)
        )
        self._hand_filter = ConstantVelocityKalman3D(
            float(self.get_parameter("kalman_process_noise").value),
            float(self.get_parameter("kalman_measurement_noise").value),
            float(self.get_parameter("kalman_innovation_gate_m").value),
            float(self.get_parameter("kalman_max_velocity_m_s").value),
        )
        self._prediction_lead = max(
            0.0, float(self.get_parameter("prediction_lead_sec").value)
        )
        self._depth_timeout = float(
            self.get_parameter("depth_timeout_sec").value
        )
        self._depth_stable_required = max(
            1, int(self.get_parameter("depth_stable_updates").value)
        )
        self._distance_reacquire_margin = float(
            self.get_parameter("distance_reacquire_margin_m").value
        )
        self._align_xy_gain = float(self.get_parameter("align_xy_gain").value)
        self._typing_plane_timeout = float(
            self.get_parameter("typing_plane_timeout_sec").value
        )
        self._typing_plane_stable_required = max(
            2, int(self.get_parameter("typing_plane_stable_updates").value)
        )
        self._typing_plane_alpha = clamp(
            float(self.get_parameter("typing_plane_filter_alpha").value),
            0.05,
            1.0,
        )
        self._keyboard_max_tilt = math.radians(
            float(self.get_parameter("keyboard_max_tilt_deg").value)
        )
        self._orientation_error_limit = math.radians(
            float(self.get_parameter("orientation_error_limit_deg").value)
        )
        self._visual_timeout = float(
            self.get_parameter("visual_target_timeout_sec").value
        )
        self._visual_alpha = clamp(
            float(self.get_parameter("visual_filter_alpha").value), 0.05, 1.0
        )
        self._tracking_vertical_offset = math.radians(
            float(
                self.get_parameter(
                    "tracking_center_vertical_offset_deg"
                ).value
            )
        )
        self._tracking_vertical_translation_sign = math.copysign(
            1.0,
            float(
                self.get_parameter(
                    "tracking_vertical_translation_sign"
                ).value
            ),
        )
        self._occlusion_downward_step = max(
            0.0,
            float(self.get_parameter("occlusion_downward_step_m").value),
        )
        self._tracking_horizontal_gain = clamp(
            float(self.get_parameter("tracking_horizontal_gain").value),
            0.0,
            1.0,
        )
        self._single_hand_horizontal_gain = clamp(
            float(self.get_parameter("single_hand_horizontal_gain").value),
            0.0,
            1.0,
        )
        self._visual_center_deadband = math.radians(
            float(self.get_parameter("visual_center_deadband_deg").value)
        )
        self._visual_center_stable_required = max(
            2, int(self.get_parameter("visual_center_stable_updates").value)
        )
        self._search_prediction_timeout = max(
            self._visual_timeout,
            float(self.get_parameter("search_prediction_timeout_sec").value),
        )
        self._single_hand_retreat = max(
            0.0, float(self.get_parameter("single_hand_retreat_m").value)
        )
        self._j4_image_gain = float(self.get_parameter("j4_image_gain").value)
        self._j5_image_gain = float(self.get_parameter("j5_image_gain").value)
        self._j4_image_sign = float(self.get_parameter("j4_image_sign").value)
        self._j5_image_sign = float(self.get_parameter("j5_image_sign").value)
        self._j4_low_head_allowance = math.radians(
            float(self.get_parameter("j4_low_head_allowance_deg").value)
        )
        self._wrist_level_gain = float(
            self.get_parameter("wrist_level_gain").value
        )
        self._wrist_level_error_limit = math.radians(
            float(self.get_parameter("wrist_level_error_limit_deg").value)
        )
        self._layout_timeout = float(
            self.get_parameter("hand_layout_timeout_sec").value
        )
        self._j5_layout_gain = float(
            self.get_parameter("j5_layout_gain").value
        )
        self._j5_layout_sign = float(
            self.get_parameter("j5_layout_sign").value
        )
        self._j5_layout_deadband = math.radians(
            float(self.get_parameter("j5_layout_deadband_deg").value)
        )
        self._j5_layout_step_limit = math.radians(
            float(self.get_parameter("j5_layout_step_limit_deg").value)
        )
        self._j5_camera_right_stop = math.radians(
            float(self.get_parameter("j5_camera_right_stop_deg").value)
        )
        self._j5_search_right_limit = math.radians(
            float(self.get_parameter("j5_search_right_limit_deg").value)
        )
        self._j5_stop_tolerance = math.radians(
            max(
                0.1,
                float(self.get_parameter("j5_stop_tolerance_deg").value),
            )
        )
        self._single_hand_j5_search_sign = math.copysign(
            1.0,
            float(self.get_parameter("single_hand_j5_search_sign").value),
        )
        self._single_hand_j5_search_step = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter("single_hand_j5_search_step_deg").value
                ),
            )
        )
        self._single_hand_j5_search_limit = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter("single_hand_j5_search_limit_deg").value
                ),
            )
        )
        self._single_hand_j5_reverse_limit = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        "single_hand_j5_reverse_limit_deg"
                    ).value
                ),
            )
        )
        self._single_hand_j5_start_delay = max(
            0.0,
            float(
                self.get_parameter("single_hand_j5_start_delay_sec").value
            ),
        )
        self._use_demonstrated_orientation = bool(
            self.get_parameter("use_demonstrated_orientation").value
        )
        self._demonstrated_joints = tuple(
            math.radians(
                float(
                    self.get_parameter(
                        f"demonstrated_joint{index}_deg"
                    ).value
                )
            )
            for index in range(1, 6)
        )
        self._depth_tracker = ConfirmedAlphaBeta1D(
            alpha=0.68,
            beta=0.10,
            jump_gate_m=float(
                self.get_parameter("depth_jump_gate_m").value
            ),
            jump_match_m=0.055,
            jump_confirmations=2,
            max_velocity_m_s=0.8,
        )
        self._depth_time = 0.0
        self._depth_in_range_count = 0
        self._servo_phase = "distance_acquire"
        self._hand_frame = ""
        self._last_filtered_hand = None
        self._last_filtered_hand_base = None
        self._last_distance_action = "waiting"
        self._last_position_error = None
        self._last_joint_delta = None
        self._last_gaze_error_deg = 0.0
        self._typing_plane_rotation = None
        self._typing_plane_time = 0.0
        self._typing_plane_updates = 0
        self._last_keyboard_tilt_deg = 0.0
        self._last_tray_reference_error_deg = 0.0
        self._j4_image_delta_deg = 0.0
        self._visual_target_camera = None
        self._visual_target_time = 0.0
        self._visual_target_updates = 0
        self._visible_hand_count = 0
        self._zero_hands_since = 0.0
        self._search_filter = ConstantVelocityKalman3D(
            float(self.get_parameter("kalman_process_noise").value),
            float(self.get_parameter("kalman_measurement_noise").value),
            float(self.get_parameter("kalman_innovation_gate_m").value),
            float(self.get_parameter("kalman_max_velocity_m_s").value),
        )
        self._hand_layout = {}
        self._hand_layout_time = 0.0
        self._using_prediction = False
        self._j5_layout_active = False
        self._j5_layout_error_deg = 0.0
        self._j5_command_delta_deg = 0.0
        self._last_j5_search_direction = self._single_hand_j5_search_sign
        self._single_hand_j5_search_origin = None
        self._j5_search_mode = "hold"
        self._j5_sweep_direction = self._single_hand_j5_search_sign
        self._j5_sweep_reversed = False
        self._j5_finalizing = False
        self._j5_pair_lost_since = 0.0
        self._visual_center_count = 0
        self._perception_hold_active = False
        self._last_servo_log_time = 0.0

        self._feedback: Optional[Tuple[float, ...]] = None
        self._feedback_time = 0.0
        self._candidate = None
        self._candidate_since = 0.0
        self._j6_lock = None
        self._anchor = None
        self._anchor_rotation = None
        self._anchor_tray_normal = None
        self._demonstrated_rotation = None
        self._demonstrated_tray_normal = None
        self._command = None
        self._command_velocity = np.zeros(6)
        self._target: Optional[np.ndarray] = None
        self._target_time = 0.0
        self._arm_seen = False
        self._arm_error = -1
        self._enabled = False
        self._gesture_start_latched = not self._require_start_gesture
        self._home_pose = None
        self._home_return_active = False
        self._target_hold_active = False
        self._settle_candidate_since = 0.0
        self._settle_last_ready_time = 0.0
        self._target_hold_reference = None
        self._hold_reacquire_since = 0.0
        self._target_was_acquired = False
        self._operation_state = (
            "waiting_gesture_one"
            if self._require_start_gesture
            else "waiting_target"
        )
        self._fault = ""
        self._warned_zero_stamp = False
        self._lock_time = 0.0
        self._nudge_started = False
        self._nudge_active = False
        self._nudge_done = self._nudge_angle <= 0.0
        self._nudge_phase = ""
        self._nudge_origin = None
        self._nudge_goal = None
        self._nudge_start_time = 0.0
        self._last_auto_wait_signature = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._command_pub = self.create_publisher(JointState, "/joint_ctrl_single", 1)
        self._preview_pub = self.create_publisher(
            JointState, "/piper_x_hand_follow/preview_joint_states", 10
        )
        self._status_pub = self.create_publisher(String, "/piper_x_hand_follow/status", 10)
        self.create_subscription(
            JointState, "/joint_states_single", self._feedback_cb, 10
        )
        self.create_subscription(
            PoseStamped,
            "/smart_keyboard/ergonomic_target_preview",
            self._target_cb,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/human_tracking/typing_plane",
            self._typing_plane_cb,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/human_tracking/visual_hand_target",
            self._visual_target_cb,
            10,
        )
        self.create_subscription(
            UInt8,
            "/human_tracking/visible_hand_count",
            self._visible_hand_count_cb,
            10,
        )
        self.create_subscription(
            String,
            "/human_tracking/hand_layout",
            self._hand_layout_cb,
            10,
        )
        self.create_subscription(
            String,
            "/human_tracking/gesture_command",
            self._gesture_command_cb,
            10,
        )
        self.create_subscription(
            Float32,
            "/human_tracking/hand_distance_m",
            self._depth_cb,
            10,
        )
        self.create_subscription(PiperStatusMsg, "/arm_status", self._arm_cb, 10)
        self.create_service(
            SetBool, "/piper_x_hand_follow/enable_tracking", self._enable_cb
        )
        self.create_timer(1.0 / self._rate, self._tick)
        self.create_timer(0.5, self._publish_status)
        mode = "REAL-CAPABLE" if self._actual_motion else "PREVIEW"
        self.get_logger().warn(
            f"{mode}; J1-J5 bounded Cartesian servo starts disabled; J6 is immutable."
        )
        self.get_logger().warn(
            f"SERVO CONFIG: {self._rate:.1f} Hz, "
            f"hand distance={self._hand_distance_range[0]:.2f}-"
            f"{self._hand_distance_range[1]:.2f} m, "
            f"joint speed<={math.degrees(self._max_joint_velocity):.1f} deg/s, "
            f"joint accel<={math.degrees(self._max_joint_acceleration):.1f} deg/s^2, "
            f"J1-J5 envelopes={np.round(np.degrees(self._max_offsets), 1).tolist()} deg."
        )

    def _feedback_cb(self, msg: JointState) -> None:
        joints = ordered_joints(msg.name, msg.position)
        if joints is None:
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            if not self._warned_zero_stamp:
                self.get_logger().warn("Ignoring zero-timestamp startup joint feedback.")
                self._warned_zero_stamp = True
            return
        now = time.monotonic()
        self._feedback = joints
        self._feedback_time = now
        if self._j6_lock is None:
            if self._candidate is None or max(
                abs(a - b) for a, b in zip(joints, self._candidate)
            ) > self._latch_span:
                self._candidate = joints
                self._candidate_since = now
                return
            if now - self._candidate_since < self._latch_settle:
                return
            self._j6_lock = joints[5]
            self._command = joints
            self._command_velocity[:] = 0.0
            self._anchor = joints
            self._home_pose = tuple(joints)
            _, self._anchor_rotation, _ = self._chain.pose_and_jacobian(joints)
            self._anchor_tray_normal = -self._anchor_rotation[:, 0].copy()
            if self._use_demonstrated_orientation:
                demonstrated_pose = self._demonstrated_joints + (
                    self._j6_lock,
                )
                _, self._demonstrated_rotation, _ = (
                    self._chain.pose_and_jacobian(demonstrated_pose)
                )
                self._demonstrated_tray_normal = (
                    -self._demonstrated_rotation[:, 0].copy()
                )
            self._lock_time = now
            self.get_logger().warn(
                f"J6 HARD LOCK latched at {math.degrees(self._j6_lock):.3f} deg."
            )
            if self._demonstrated_tray_normal is not None:
                self.get_logger().warn(
                    "Using real teaching orientation from "
                    "manual_teach_20260725_213926 (J1-J5="
                    f"{[round(math.degrees(q), 1) for q in self._demonstrated_joints]} deg)."
                )
        elif abs(joints[5] - self._j6_lock) > self._j6_limit:
            self._set_fault(
                f"J6 drift: lock={math.degrees(self._j6_lock):.3f} deg, "
                f"feedback={math.degrees(joints[5]):.3f} deg"
            )

    def _target_cb(self, msg: PoseStamped) -> None:
        if msg.header.frame_id != "base_link":
            return
        target = np.asarray(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        if np.all(np.isfinite(target)):
            self._target = target
            self._target_time = time.monotonic()

    def _hand_cb(self, msg: PointStamped) -> None:
        point = np.asarray([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        if not np.all(np.isfinite(point)):
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, Time()
            )
        except Exception:
            return
        q = transform.transform.rotation
        rotation = _quaternion_matrix(q.x, q.y, q.z, q.w)
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        )
        point_base = rotation @ point + translation
        now = time.monotonic()
        self._hand_frame = msg.header.frame_id
        self._camera_frame = msg.header.frame_id
        self._target_time = now

    def _typing_plane_cb(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        if (
            self._typing_plane_time > 0.0
            and now - self._typing_plane_time > self._typing_plane_timeout
        ):
            self._typing_plane_rotation = None
            self._typing_plane_updates = 0
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, Time()
            )
        except Exception:
            return
        tf_q = transform.transform.rotation
        rotation_base_from_source = _quaternion_matrix(
            tf_q.x, tf_q.y, tf_q.z, tf_q.w
        )
        pose_q = msg.pose.orientation
        rotation_source_from_plane = _quaternion_matrix(
            pose_q.x, pose_q.y, pose_q.z, pose_q.w
        )
        measured_rotation = (
            rotation_base_from_source @ rotation_source_from_plane
        )
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        )
        point_source = np.asarray(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
        )
        point_base = rotation_base_from_source @ point_source + translation
        if not np.all(np.isfinite(point_base)):
            return
        if self._typing_plane_rotation is None:
            filtered = measured_rotation
        else:
            previous = self._typing_plane_rotation
            lateral = measured_rotation[:, 0].copy()
            normal = measured_rotation[:, 2].copy()
            if float(np.dot(lateral, previous[:, 0])) < 0.0:
                lateral = -lateral
            if float(np.dot(normal, previous[:, 2])) < 0.0:
                normal = -normal
            alpha = self._typing_plane_alpha
            normal = (1.0 - alpha) * previous[:, 2] + alpha * normal
            normal /= np.linalg.norm(normal)
            lateral = (1.0 - alpha) * previous[:, 0] + alpha * lateral
            lateral -= normal * float(np.dot(lateral, normal))
            lateral_norm = float(np.linalg.norm(lateral))
            if lateral_norm < 1.0e-6:
                return
            lateral /= lateral_norm
            forward = np.cross(normal, lateral)
            forward /= np.linalg.norm(forward)
            filtered = np.column_stack((lateral, forward, normal))
        self._typing_plane_rotation = filtered
        self._typing_plane_time = now
        self._typing_plane_updates += 1
        self._hand_filter.update(point_base, now)
        self._hand_frame = msg.header.frame_id
        self._camera_frame = msg.header.frame_id
        self._target_time = now

    def _typing_plane_ready(self, now: Optional[float] = None) -> bool:
        current_time = time.monotonic() if now is None else now
        return (
            self._typing_plane_rotation is not None
            and self._typing_plane_updates >= self._typing_plane_stable_required
            and current_time - self._typing_plane_time <= self._typing_plane_timeout
        )

    def _visual_target_cb(self, msg: PointStamped) -> None:
        point = np.asarray([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        if (
            not np.all(np.isfinite(point))
            or point[2] <= 0.0
            or not msg.header.frame_id
        ):
            return
        now = time.monotonic()
        if (
            self._visual_target_camera is None
            or now - self._visual_target_time > self._visual_timeout
        ):
            filtered = point
            self._visual_target_updates = 1
        else:
            filtered = (
                (1.0 - self._visual_alpha) * self._visual_target_camera
                + self._visual_alpha * point
            )
            self._visual_target_updates += 1
        self._visual_target_camera = filtered
        self._visual_target_time = now
        self._camera_frame = msg.header.frame_id
        try:
            transform = self._tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, Time()
            )
            q = transform.transform.rotation
            rotation = _quaternion_matrix(q.x, q.y, q.z, q.w)
            translation = np.asarray(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )
            self._search_filter.update(rotation @ point + translation, now)
        except Exception:
            # Current camera-frame tracking remains available until TF catches up.
            pass

    def _visible_hand_count_cb(self, msg: UInt8) -> None:
        count = int(msg.data)
        self._visible_hand_count = max(0, min(2, count))
        if self._visible_hand_count == 0:
            self._visual_center_count = 0
            if self._zero_hands_since <= 0.0:
                self._zero_hands_since = time.monotonic()
        else:
            self._zero_hands_since = 0.0

    def _hand_layout_cb(self, msg: String) -> None:
        try:
            layout = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(layout, dict):
            self._hand_layout = layout
            self._hand_layout_time = time.monotonic()

    def _gesture_command_cb(self, msg: String) -> None:
        if not self._require_start_gesture:
            return
        command = str(msg.data).strip().lower()
        if command == "start":
            if self._home_return_active:
                return
            start_failures = self._failures()
            if start_failures:
                # Remember an intentional gesture instead of forcing the user
                # to repeat it after joint/depth startup finishes. The normal
                # auto-start path still refuses motion until every failure has
                # cleared, including a current (not predicted) visible hand.
                self._gesture_start_latched = True
                self._operation_state = "start_requested"
                self.get_logger().warning(
                    "GESTURE 1 queued; motion will start when ready: "
                    + "; ".join(start_failures)
                )
                return
            self._gesture_start_latched = True
            self._target_hold_active = False
            self._settle_candidate_since = 0.0
            self._settle_last_ready_time = 0.0
            self._target_hold_reference = None
            self._hold_reacquire_since = 0.0
            self._target_was_acquired = False
            self._operation_state = (
                "tracking" if self._enabled else "start_requested"
            )
            self.get_logger().warning(
                "GESTURE 1 accepted: hand tracking may start when perception is ready."
            )
            return
        if command != "home" or self._home_pose is None:
            return
        self._begin_home_return("GESTURE 2 accepted")

    def _begin_home_return(self, reason: str) -> None:
        if self._home_pose is None or self._feedback is None:
            return
        self._enabled = False
        self._gesture_start_latched = False
        self._home_return_active = True
        self._target_hold_active = False
        self._settle_candidate_since = 0.0
        self._settle_last_ready_time = 0.0
        self._target_hold_reference = None
        self._hold_reacquire_since = 0.0
        self._target_was_acquired = False
        self._zero_hands_since = 0.0
        self._operation_state = "returning_home"
        self._command = tuple(self._feedback or self._home_pose)
        self._command_velocity[:] = 0.0
        self._single_hand_j5_search_origin = None
        self.get_logger().warning(
            f"{reason}: visual tracking stopped; returning J1-J5 "
            "to the startup pose."
        )

    def _visual_target_ready(self, now: Optional[float] = None) -> bool:
        current_time = time.monotonic() if now is None else now
        return (
            self._visual_target_camera is not None
            and self._visible_hand_count > 0
            and self._visual_target_updates >= 2
            and current_time - self._visual_target_time <= self._visual_timeout
        )

    def _predicted_search_ready(self, now: Optional[float] = None) -> bool:
        current_time = time.monotonic() if now is None else now
        return (
            self._search_filter.initialized
            and self._search_filter.update_count >= self._kalman_min_updates
            and self._search_filter.last_measurement_time is not None
            and current_time - self._search_filter.last_measurement_time
            <= self._search_prediction_timeout
        )

    def _search_target_ready(self, now: Optional[float] = None) -> bool:
        # Prediction is useful in RViz preview, but must never authorize
        # physical motion after the camera has lost every hand.
        return self._visual_target_ready(now) or (
            not self._actual_motion and self._predicted_search_ready(now)
        )

    def _depth_cb(self, msg: Float32) -> None:
        now = time.monotonic()
        try:
            accepted = self._depth_tracker.update(float(msg.data), now)
        except ValueError:
            return
        if not accepted:
            return
        self._depth_time = now

    def _arm_cb(self, msg: PiperStatusMsg) -> None:
        self._arm_seen = True
        self._arm_error = int(msg.err_code)
        if self._arm_error != 0:
            self._set_fault(f"arm_status err_code={self._arm_error}")

    def _failures(self):
        now = time.monotonic()
        failures = []
        if self._fault:
            failures.append(f"fault={self._fault}")
        if self._feedback is None or self._j6_lock is None:
            failures.append("real joint feedback/J6 lock not ready")
        elif now - self._feedback_time > self._feedback_timeout:
            failures.append("joint feedback stale")
        if self._servo_mode == "eye_in_hand":
            if not self._search_target_ready(now):
                failures.append("current or predicted hand target not ready")
            elif self._visual_target_ready(now) and (
                not self._depth_tracker.initialized
                or now - self._depth_time > self._depth_timeout
            ):
                failures.append("robust hand depth not ready")
        elif (
            self._target is None
            or now - self._target_time > self._target_timeout
        ):
            failures.append("ergonomic target not ready")
        if not self._arm_seen:
            failures.append("arm_status not received")
        elif self._arm_error != 0:
            failures.append(f"arm err_code={self._arm_error}")
        if not self._mount_confirmed:
            failures.append("link6-to-camera mount transform not confirmed")
        return failures

    def _enable_cb(self, request, response):
        if not request.data:
            self._enabled = False
            response.success = True
            response.message = "tracking disabled; motors remain enabled and hold pose"
            return response
        failures = self._failures()
        if self._require_start_gesture and not self._gesture_start_latched:
            failures.append("gesture 1 not received")
        if not self._actual_motion:
            failures.append("actual_motion=false")
        if failures:
            response.success = False
            response.message = "; ".join(failures)
            return response
        self._anchor = self._feedback
        _, self._anchor_rotation, _ = self._chain.pose_and_jacobian(self._anchor)
        self._anchor_tray_normal = -self._anchor_rotation[:, 0].copy()
        self._command = self._feedback
        self._command_velocity[:] = 0.0
        self._enabled = True
        response.success = True
        response.message = (
            f"Cartesian hand tracking enabled; J6={math.degrees(self._j6_lock):.3f} deg"
        )
        return response

    def _tick(self) -> None:
        if self._feedback is None or self._j6_lock is None:
            return
        if self._anchor is None:
            return
        if self._home_return_active:
            self._home_return_tick()
            return
        now = time.monotonic()
        if hand_loss_home_due(
            self._target_was_acquired,
            self._zero_hands_since,
            now,
            self._hand_loss_home_sec,
        ):
            self._begin_home_return(
                f"HANDS LOST for {self._hand_loss_home_sec:.1f} s"
            )
            self._home_return_tick()
            return
        if self._target_hold_active:
            self._target_hold_tick()
            return
        self._maybe_start_nudge()
        if self._nudge_active:
            self._nudge_tick()
            return
        if (
            self._auto_start
            and self._nudge_done
            and not self._enabled
            and self._gesture_start_latched
        ):
            if not self._failures():
                self._anchor = self._feedback
                _, self._anchor_rotation, _ = self._chain.pose_and_jacobian(
                    self._anchor
                )
                self._anchor_tray_normal = -self._anchor_rotation[:, 0].copy()
                self._command = self._feedback
                self._enabled = True
                self._operation_state = "tracking"
                self.get_logger().warn(
                    "Current hand target ready: visual search started automatically."
                )
        if self._servo_mode == "ergonomic_base" and self._target is None:
            return
        if self._enabled:
            failures = self._failures()
            hard_failures = [
                reason
                for reason in failures
                if reason
                not in (
                    "robust hand depth not ready",
                    "current or predicted hand target not ready",
                )
            ]
            if hard_failures:
                self._enabled = False
                self._operation_state = "fault_hold"
                return
            if failures:
                # Cancel the driver's previous distant goal. Merely stopping
                # publication is insufficient because the PIPER driver keeps
                # executing its last JointState target.
                if not self._perception_hold_active:
                    self._command = tuple(self._feedback)
                    self._command_velocity[:] = 0.0
                    self._preview_pub.publish(
                        self._joint_message(self._command, False)
                    )
                    if self._actual_motion:
                        self._command_pub.publish(
                            self._joint_message(self._command, True)
                        )
                    self._last_distance_action = "hold_no_current_hand"
                    self._perception_hold_active = True
                return
            self._perception_hold_active = False
        tcp_position, position_jacobian = self._chain.offset_point_and_jacobian(
            self._feedback, self._tcp
        )
        _, current_link_rotation, geometric_jacobian = (
            self._chain.pose_and_jacobian(self._feedback)
        )
        # Defaults keep the non-eye-in-hand preview path well-defined.  The
        # real eye-in-hand branch replaces these from live perception below.
        current_visual = False
        downward_occlusion_search = False
        settle_position_ready = False
        settle_geometry_ready = False
        if self._servo_mode == "eye_in_hand":
            now = time.monotonic()
            try:
                transform = self._tf_buffer.lookup_transform(
                    "base_link", self._camera_frame, Time()
                )
            except Exception:
                return
            q = transform.transform.rotation
            rotation_base_from_camera = _quaternion_matrix(q.x, q.y, q.z, q.w)
            camera_origin_base = np.asarray(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )
            minimum_distance, maximum_distance = self._hand_distance_range
            desired_distance = clamp(
                float(self._desired_hand[2]),
                minimum_distance,
                maximum_distance,
            )
            current_visual = self._visual_target_ready(now)
            if current_visual:
                hand_point = self._visual_target_camera.copy()
            elif not self._actual_motion and self._predicted_search_ready(now):
                predicted_base = self._search_filter.forecast(
                    now, self._prediction_lead
                )
                hand_point = (
                    rotation_base_from_camera.T
                    @ (predicted_base - camera_origin_base)
                )
            else:
                return
            if current_visual and self._depth_tracker.initialized:
                depth = self._depth_tracker.forecast(
                    now, self._prediction_lead
                )
                hand_point[2] = depth
            else:
                depth = float(hand_point[2])
            if not np.all(np.isfinite(hand_point)) or hand_point[2] <= 0.05:
                return
            horizontal_angle = math.atan2(hand_point[0], max(depth, 1.0e-6))
            vertical_angle = math.atan2(hand_point[1], max(depth, 1.0e-6))
            vertical_error = (
                vertical_angle - self._tracking_vertical_offset
            )
            downward_occlusion_search = (
                vertical_error > self._visual_center_deadband
            )
            center_error = max(abs(horizontal_angle), abs(vertical_error))
            desired_vertical_y = (
                math.tan(self._tracking_vertical_offset) * depth
            )
            servo_hand_point = hand_point.copy()
            servo_hand_point[1] -= desired_vertical_y
            both_hands = self._visible_hand_count >= 2
            centered = center_error <= self._visual_center_deadband
            if both_hands and centered:
                self._visual_center_count += 1
            else:
                self._visual_center_count = 0

            current_tray_normal = -current_link_rotation[:, 0]
            final_tray_normal = (
                self._demonstrated_tray_normal
                if self._demonstrated_tray_normal is not None
                else self._anchor_tray_normal
            )
            final_orientation_error = (
                math.acos(
                    clamp(
                        float(
                            np.dot(
                                current_tray_normal,
                                final_tray_normal,
                            )
                        ),
                        -1.0,
                        1.0,
                    )
                )
                if final_tray_normal is not None
                else 0.0
            )
            self._last_tray_reference_error_deg = math.degrees(
                final_orientation_error
            )
            settle_position_ready = (
                self._settle_hold_enabled
                and current_visual
                and self._visible_hand_count >= 1
                and center_error <= self._settle_center_error
                and minimum_distance <= depth <= maximum_distance
            )
            if settle_position_ready:
                # The endpoint has reached the user's usable centre/distance
                # zone. Orientation may still be converging, but a later
                # continuous six-second total hand loss should return home.
                self._target_was_acquired = True
            settle_geometry_ready = (
                settle_position_ready
                and final_orientation_error
                <= self._settle_orientation_error
            )
            self._j5_finalizing = (
                settle_position_ready
                and abs(
                    self._feedback[4] - self._j5_camera_right_stop
                )
                > self._j5_stop_tolerance
            )
            settle_ready = settle_geometry_ready and not self._j5_finalizing
            if self._j5_finalizing:
                self._settle_candidate_since = 0.0
                self._settle_last_ready_time = 0.0
            elif settle_ready:
                self._settle_last_ready_time = now
                if self._settle_candidate_since <= 0.0:
                    self._settle_candidate_since = now
                elif now - self._settle_candidate_since >= self._settle_hold_time:
                    self._enter_target_hold()
                    return
            elif (
                self._settle_candidate_since <= 0.0
                or now - self._settle_last_ready_time
                > self._settle_dropout_grace
            ):
                self._settle_candidate_since = 0.0
                self._settle_last_ready_time = 0.0

            self._using_prediction = not current_visual
            if self._using_prediction:
                # Continue in the last measured direction in all image axes,
                # but never approach blindly when the hand is outside view.
                camera_error = np.asarray(
                    [hand_point[0], servo_hand_point[1], 0.0]
                )
                self._servo_phase = "predictive_search"
                self._last_distance_action = "kalman_find_hand"
            else:
                (
                    camera_error,
                    self._servo_phase,
                    self._last_distance_action,
                ) = visual_search_command(
                    servo_hand_point,
                    self._visible_hand_count,
                    self._visual_center_count
                    >= self._visual_center_stable_required,
                    desired_distance,
                    minimum_distance,
                    maximum_distance,
                    self._single_hand_retreat,
                )

            # Eye-in-hand image motion is opposite camera translation. The
            # measured PIPER-X mount requires a negative physical sign: when a
            # hand is below the raised image target, move the endpoint down so
            # the hand rises in the image instead of lifting the keyboard into
            # the camera view.
            horizontal_gain = (
                self._single_hand_horizontal_gain
                if self._visible_hand_count < 2
                else self._tracking_horizontal_gain
            )
            camera_error[0] *= horizontal_gain
            camera_error[1] *= self._tracking_vertical_translation_sign
            camera_error = np.asarray(
                [
                    0.0 if abs(value) <= deadband else value
                    for value, deadband in zip(
                        camera_error, self._hand_deadband
                    )
                ]
            )
            position_error = rotation_base_from_camera @ camera_error
            self._last_filtered_hand = hand_point.copy()
            self._last_filtered_hand_base = None
        else:
            position_error = self._target - tcp_position
        norm = float(np.linalg.norm(position_error))
        if norm > self._max_target_delta:
            position_error *= self._max_target_delta / norm
        if norm > self._max_cart_step:
            position_error *= self._max_cart_step / max(float(np.linalg.norm(position_error)), 1e-9)
        if self._servo_mode == "eye_in_hand":
            # Apply after the Cartesian magnitude clamp so a large horizontal
            # error cannot scale the required downward component back to zero.
            position_error = enforce_base_downward_trend(
                position_error,
                downward_occlusion_search,
                self._occlusion_downward_step,
            )
            if downward_occlusion_search:
                self._last_distance_action += "_base_down"
        position_error = workspace_limited_step(
            tcp_position,
            position_error,
            self._workspace[0],
            self._workspace[1],
            self._workspace[2],
            self._workspace[3],
        )
        # Solve translation and tray-level constraints together. Restricting
        # translation to J1-J3 made the real arm singular near its straight
        # startup pose: J3 stayed around zero while J4/J5 tried to find hands
        # by rotating the camera. The coordinated solve lets J2/J3 lower the
        # TCP and uses J4 only to compensate the keyboard pitch.
        current_tray_normal = -current_link_rotation[:, 0]
        orientation_tracking_ready = (
            self._servo_mode != "eye_in_hand"
            or (
                current_visual
                and center_error <= math.radians(15.0)
                and depth <= maximum_distance + 0.08
            )
        )
        desired_tray_normal = (
            self._demonstrated_tray_normal
            if (
                self._demonstrated_tray_normal is not None
                and orientation_tracking_ready
            )
            else (
                self._anchor_tray_normal
                if self._anchor_tray_normal is not None
                else current_tray_normal
            )
        )
        self._j5_layout_active = False
        self._j5_layout_error_deg = 0.0
        self._j5_command_delta_deg = 0.0
        self._j5_search_mode = "hold"
        layout_fresh = (
            time.monotonic() - self._hand_layout_time
            <= self._layout_timeout
        )
        hand_line_angle = self._hand_layout.get("hand_line_angle_rad")
        hand_line_valid = isinstance(hand_line_angle, (int, float)) and math.isfinite(
            float(hand_line_angle)
        )
        valid_pair_layout = (
            self._servo_mode == "eye_in_hand"
            and not settle_position_ready
            and layout_fresh
            and self._visible_hand_count >= 2
            and self._hand_layout.get("usable_pair", False)
            and hand_line_valid
        )
        if valid_pair_layout:
            self._j5_pair_lost_since = 0.0
        elif self._j5_pair_lost_since <= 0.0:
            self._j5_pair_lost_since = now
        pair_loss_stable = (
            self._j5_pair_lost_since > 0.0
            and now - self._j5_pair_lost_since
            >= self._single_hand_j5_start_delay
        )
        if valid_pair_layout:
            j5_step = demonstrated_j5_layout_step(
                float(hand_line_angle),
                gain=self._j5_layout_gain,
                physical_sign=self._j5_layout_sign,
                deadband_rad=self._j5_layout_deadband,
                maximum_step_rad=self._j5_layout_step_limit,
            )
            self._j5_layout_active = True
            self._j5_layout_error_deg = math.degrees(float(hand_line_angle))
            self._j5_command_delta_deg = math.degrees(j5_step)
            self._j5_search_mode = "two_hand_layout"
            self._single_hand_j5_search_origin = None
            self._j5_sweep_reversed = False
            if abs(j5_step) > 1.0e-8:
                self._last_j5_search_direction = math.copysign(1.0, j5_step)
        elif (
            self._servo_mode == "eye_in_hand"
            and not settle_position_ready
            and layout_fresh
            and pair_loss_stable
            and (
                self._visible_hand_count == 1
                or (
                    self._single_hand_j5_search_origin is not None
                    and self._search_target_ready(now)
                )
            )
            and self._single_hand_j5_search_step > 0.0
            and self._single_hand_j5_search_limit > 0.0
        ):
            if self._single_hand_j5_search_origin is None:
                self._single_hand_j5_search_origin = float(self._feedback[4])
                self._j5_sweep_direction = self._last_j5_search_direction
                self._j5_sweep_reversed = False
            sweep_limit = (
                self._single_hand_j5_reverse_limit
                if self._j5_sweep_reversed
                else self._single_hand_j5_search_limit
            )
            j5_step, at_search_limit = bounded_single_hand_j5_step(
                float(self._feedback[4]),
                self._single_hand_j5_search_origin,
                self._j5_sweep_direction,
                self._single_hand_j5_search_step,
                sweep_limit,
            )
            if at_search_limit and not self._j5_sweep_reversed:
                self._j5_sweep_reversed = True
                self._j5_sweep_direction = -self._j5_sweep_direction
                j5_step, at_search_limit = bounded_single_hand_j5_step(
                    float(self._feedback[4]),
                    self._single_hand_j5_search_origin,
                    self._j5_sweep_direction,
                    self._single_hand_j5_search_step,
                    self._single_hand_j5_reverse_limit,
                )
            if not at_search_limit:
                self._j5_layout_active = True
                self._j5_command_delta_deg = math.degrees(j5_step)
                self._j5_search_mode = (
                    "single_hand_reverse_left"
                    if self._j5_sweep_reversed
                    else "single_hand_reacquire"
                )
            else:
                self._j5_search_mode = "single_hand_sweep_complete"
        # Do not reset the search origin on brief 1->0 detector flicker.
        # Otherwise repeated flicker could restart the sweep on every
        # reacquisition and defeat the bounded-search safety limit. A valid
        # two-hand layout above is the only event that rearms the search.
        orientation_gain = (
            0.0
            if (
                self._servo_mode == "eye_in_hand"
                and downward_occlusion_search
                and not settle_position_ready
            )
            else self._wrist_level_gain
        )
        delta = position_level_ik_step(
            position_jacobian,
            geometric_jacobian[3:, :],
            position_error,
            current_tray_normal,
            desired_tray_normal=desired_tray_normal,
            orientation_weight_m_per_rad=self._orientation_weight,
            orientation_gain=orientation_gain,
            damping=self._damping,
            joint_penalties=(1.0, 1.0, 1.0, 2.0),
        )
        if self._j5_layout_active:
            delta[4] = j5_step
        if self._j5_finalizing:
            delta[4] = (
                self._j5_camera_right_stop - float(self._feedback[4])
            )
            self._j5_command_delta_deg = math.degrees(float(delta[4]))
            self._j5_search_mode = "final_to_right_facing_target"
        self._j4_image_delta_deg = 0.0
        if self._servo_mode == "eye_in_hand":
            # Physical PIPER-X observation: positive J4 pitches the camera
            # downward. Do not let a guessed world-up convention reverse it.
            j4_image_step = physical_j4_vertical_search_step(
                vertical_error,
                self._j4_image_gain,
                self._j4_image_sign,
                self._visual_center_deadband,
                math.radians(3.0),
            )
            delta[3] += j4_image_step
            self._j4_image_delta_deg = math.degrees(j4_image_step)
        self._last_gaze_error_deg = 0.0
        if self._servo_mode == "eye_in_hand":
            # Horizontal midpoint errors are never sent directly to J5.
            # The small yaw term above exists only when two complete hand
            # skeletons define a trustworthy typing-plane lateral axis.
            self._last_gaze_error_deg = math.degrees(center_error)
        actual_tray_normal = -current_link_rotation[:, 0]
        self._last_keyboard_tilt_deg = math.degrees(
            math.acos(clamp(float(actual_tray_normal[2]), -1.0, 1.0))
        )
        self._last_position_error = position_error.copy()
        self._last_joint_delta = delta.copy()
        target_joints = list(self._feedback)
        for index in range(5):
            lower = max(
                _SAFE_BOUNDS[index][0],
                self._anchor[index] - self._max_offsets[index],
            )
            upper = min(
                _SAFE_BOUNDS[index][1],
                self._anchor[index] + self._max_offsets[index],
            )
            if index == 4:
                # Searching may pass the 52-degree final threshold, but must
                # never repeat the large 70-80 degree excursions seen on the
                # physical arm.
                upper = min(upper, self._j5_search_right_limit)
            target_joints[index] = clamp(
                self._feedback[index] + float(delta[index]), lower, upper
            )
        target_joints[5] = self._j6_lock
        current = self._command if self._command is not None else self._feedback
        self._command = self._smooth_command(current, target_joints)
        self._preview_pub.publish(self._joint_message(self._command, False))
        if self._enabled and self._actual_motion:
            self._command_pub.publish(self._joint_message(self._command, True))

    def _maybe_start_nudge(self) -> None:
        if (
            self._nudge_started
            or self._nudge_done
            or not self._actual_motion
            or not self._arm_seen
            or self._arm_error != 0
            or time.monotonic() - self._lock_time < 1.0
        ):
            return
        self._nudge_started = True
        self._nudge_active = True
        self._nudge_phase = "out"
        self._nudge_origin = tuple(self._feedback)
        direction = 1.0
        if self._feedback[0] + self._nudge_angle > _SAFE_BOUNDS[0][1]:
            direction = -1.0
        goal = list(self._feedback)
        goal[0] += direction * self._nudge_angle
        goal[5] = self._j6_lock
        self._nudge_goal = tuple(goal)
        self._command = tuple(self._feedback)
        self._command_velocity[:] = 0.0
        self._nudge_start_time = time.monotonic()
        self.get_logger().warn(
            f"STARTUP MOTION: J1 {direction * math.degrees(self._nudge_angle):+.3f} deg "
            "then return; J6 remains hard locked."
        )

    def _nudge_tick(self) -> None:
        goal = self._nudge_goal if self._nudge_phase == "out" else self._nudge_origin
        if time.monotonic() - self._nudge_start_time > self._nudge_timeout:
            self._nudge_active = False
            self._set_fault(
                "startup J1 nudge timed out: "
                f"phase={self._nudge_phase}, "
                f"feedback={math.degrees(self._feedback[0]):.3f} deg, "
                f"goal={math.degrees(goal[0]):.3f} deg"
            )
            return
        self._command = self._smooth_command(self._command, goal)
        self._preview_pub.publish(self._joint_message(self._command, False))
        self._command_pub.publish(self._joint_message(self._command, True))
        tolerance = math.radians(0.20)
        if abs(self._feedback[0] - goal[0]) > tolerance:
            return
        if self._nudge_phase == "out":
            self._nudge_phase = "return"
            self.get_logger().warn("STARTUP MOTION: returning J1 to its initial angle.")
            return
        self._nudge_active = False
        self._nudge_done = True
        self._anchor = tuple(self._feedback)
        _, self._anchor_rotation, _ = self._chain.pose_and_jacobian(self._anchor)
        self._anchor_tray_normal = -self._anchor_rotation[:, 0].copy()
        self._command = tuple(self._feedback)
        self._command_velocity[:] = 0.0
        self.get_logger().warn(
            "STARTUP MOTION complete; waiting for a stable 3D hand target."
        )

    def _home_return_tick(self) -> None:
        if self._home_pose is None or self._feedback is None:
            return
        if (
            self._fault
            or self._arm_error != 0
            or time.monotonic() - self._feedback_time > self._feedback_timeout
        ):
            self._home_return_active = False
            self._operation_state = "home_fault_hold"
            self.get_logger().error(
                "HOME RETURN stopped because feedback or arm safety is not healthy."
            )
            return
        current = self._command if self._command is not None else self._feedback
        target = list(self._home_pose)
        target[5] = self._j6_lock
        self._command = self._smooth_command(
            current,
            target,
            speed_scale=self._home_return_speed_scale,
        )
        message = self._joint_message(
            self._command,
            self._actual_motion,
            speed_percent=max(
                1,
                int(round(self._speed_percent * self._home_return_speed_scale)),
            ),
        )
        self._preview_pub.publish(message)
        if self._actual_motion:
            self._command_pub.publish(message)
        if max(
            abs(float(a) - float(b))
            for a, b in zip(self._feedback[:5], self._home_pose[:5])
        ) > self._home_tolerance:
            return
        self._home_return_active = False
        self._command = tuple(target)
        self._command_velocity[:] = 0.0
        self._operation_state = (
            "waiting_gesture_one"
            if self._require_start_gesture
            else "home"
        )
        self.get_logger().warning(
            "HOME RETURN complete: J1-J5 restored; waiting for gesture 1."
        )

    def _enter_target_hold(self) -> None:
        """Cancel the moving driver goal and latch the reached hand pose."""
        if self._feedback is None:
            return
        self._command = tuple(self._feedback)
        self._command_velocity[:] = 0.0
        hold_message = self._joint_message(self._command, self._actual_motion)
        self._preview_pub.publish(hold_message)
        if self._actual_motion:
            self._command_pub.publish(hold_message)
        self._target_hold_active = True
        self._target_was_acquired = True
        self._target_hold_reference = (
            self._visual_target_camera.copy()
            if self._visual_target_camera is not None
            else None
        )
        self._hold_reacquire_since = 0.0
        self._operation_state = "target_hold"
        self._last_distance_action = "stable_target_hold"
        self._settle_candidate_since = 0.0
        self._settle_last_ready_time = 0.0
        self.get_logger().warning(
            "HAND TARGET ACQUIRED: approximate centre and safe distance "
            "stable for the configured dwell; J1-J5 pose is now held. "
            "A sustained large hand displacement or gesture 1 reacquires; "
            "gesture 2 returns home."
        )

    def _target_hold_tick(self) -> None:
        """Keep the reached pose still until deliberate hand motion persists."""
        now = time.monotonic()
        if (
            self._target_hold_reference is None
            or not self._visual_target_ready(now)
            or self._visible_hand_count < 2
        ):
            self._hold_reacquire_since = 0.0
            return
        changed = hold_target_changed(
            self._target_hold_reference,
            self._visual_target_camera,
            self._hold_reacquire_xy,
            self._hold_reacquire_depth,
        )
        if not changed:
            self._hold_reacquire_since = 0.0
            return
        if self._hold_reacquire_since <= 0.0:
            self._hold_reacquire_since = now
            return
        if now - self._hold_reacquire_since < self._hold_reacquire_time:
            return
        self._target_hold_active = False
        self._target_hold_reference = None
        self._hold_reacquire_since = 0.0
        self._settle_candidate_since = 0.0
        self._settle_last_ready_time = 0.0
        self._command = tuple(self._feedback)
        self._command_velocity[:] = 0.0
        self._operation_state = "tracking"
        self._last_distance_action = "large_hand_change_reacquire"
        self.get_logger().warning(
            "LARGE HAND CHANGE sustained: target hold released; "
            "J1-J5 tracking resumed."
        )

    def _smooth_command(self, current, target, speed_scale=1.0):
        dt = 1.0 / self._rate
        speed_scale = clamp(float(speed_scale), 0.1, 1.0)
        output = list(current)
        for index in range(5):
            error = float(target[index]) - float(current[index])
            velocity_limit = speed_scale * (
                self._j5_max_joint_velocity
                if index == 4
                else self._max_joint_velocity
            )
            acceleration_limit = speed_scale * (
                self._j5_max_joint_acceleration
                if index == 4
                else self._max_joint_acceleration
            )
            velocity_step = acceleration_limit * dt
            desired_velocity = clamp(
                error / dt,
                -velocity_limit,
                velocity_limit,
            )
            velocity = clamp(
                desired_velocity,
                self._command_velocity[index] - velocity_step,
                self._command_velocity[index] + velocity_step,
            )
            step = velocity * dt
            if step * error > 0.0 and abs(step) >= abs(error):
                output[index] = float(target[index])
                velocity = 0.0
            else:
                output[index] = float(current[index]) + step
            self._command_velocity[index] = velocity
        output[5] = self._j6_lock
        self._command_velocity[5] = 0.0
        return enforce_j6_lock(output, self._j6_lock)

    def _joint_message(self, joints, real, speed_percent=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint{index}" for index in range(1, 7)]
        msg.position = list(joints)
        requested_speed = (
            self._speed_percent
            if speed_percent is None
            else int(clamp(float(speed_percent), 1.0, 100.0))
        )
        msg.velocity = (
            [0.0] * 6 + [float(requested_speed)]
            if real
            else [0.0] * 6
        )
        msg.effort = [0.0] * 6
        return msg

    def _set_fault(self, reason: str) -> None:
        if not self._fault:
            self.get_logger().error("SAFETY FAULT: " + reason)
        self._fault = reason
        self._enabled = False

    def _publish_status(self) -> None:
        if self._auto_start:
            if self._home_return_active:
                auto_state = ("returning_home",)
            elif self._target_hold_active:
                auto_state = ("target_hold",)
            elif self._require_start_gesture and not self._gesture_start_latched:
                auto_state = ("waiting_gesture_one",)
            elif self._nudge_active:
                auto_state = (f"startup_nudge_{self._nudge_phase}",)
            elif not self._nudge_done:
                auto_state = ("waiting_for_startup_nudge",)
            elif self._enabled:
                auto_state = ("tracking_active",)
            else:
                failures = self._failures()
                auto_state = tuple(failures) if failures else ("ready_to_start",)
            if auto_state != self._last_auto_wait_signature:
                self._last_auto_wait_signature = auto_state
                self.get_logger().info(
                    "AUTO TRACK STATE: " + "; ".join(auto_state)
                )
        now = time.monotonic()
        if (
            self._enabled
            and self._last_filtered_hand is not None
            and now - self._last_servo_log_time >= 1.0
        ):
            self._last_servo_log_time = now
            self.get_logger().info(
                "SERVO ACTIVE: "
                f"depth={self._depth_tracker.value:.3f} m, "
                f"depth_v={self._depth_tracker.velocity:.3f} m/s, "
                f"phase={self._servo_phase}, "
                f"action={self._last_distance_action}, "
                f"hand_camera={np.round(self._last_filtered_hand, 3).tolist()}, "
                f"center_error={self._last_gaze_error_deg:.1f} deg, "
                f"keyboard_tilt={self._last_keyboard_tilt_deg:.1f} deg, "
                f"tray_reference_error="
                f"{self._last_tray_reference_error_deg:.1f} deg, "
                f"visible_hands={self._visible_hand_count}, "
                f"prediction={self._using_prediction}, "
                f"j5_layout={self._j5_layout_active}, "
                f"j5_mode={self._j5_search_mode}, "
                f"j5_error={self._j5_layout_error_deg:.1f} deg, "
                f"j5_delta={self._j5_command_delta_deg:+.2f} deg, "
                f"j4_image_delta={self._j4_image_delta_deg:+.2f} deg, "
                f"depth_rejected={self._depth_tracker.rejected_count}, "
                f"J1-J5={np.round(np.degrees(self._feedback[:5]), 1).tolist()} deg, "
                f"J6_lock={math.degrees(self._j6_lock):.3f} deg"
            )
        msg = String()
        msg.data = json.dumps(
            {
                "mode": "real-capable" if self._actual_motion else "preview",
                "servo_mode": self._servo_mode,
                "tracking_enabled": self._enabled,
                "operation_state": self._operation_state,
                "gesture_start_latched": self._gesture_start_latched,
                "home_return_active": self._home_return_active,
                "home_return_speed_scale": self._home_return_speed_scale,
                "target_hold_active": self._target_hold_active,
                "target_was_acquired": self._target_was_acquired,
                "hand_loss_home_sec": self._hand_loss_home_sec,
                "hand_loss_elapsed_sec": round(
                    max(0.0, now - self._zero_hands_since)
                    if self._zero_hands_since > 0.0
                    else 0.0,
                    2,
                ),
                "settle_hold_enabled": self._settle_hold_enabled,
                "settle_hold_time_sec": self._settle_hold_time,
                "hold_reacquire_xy_m": self._hold_reacquire_xy,
                "hold_reacquire_depth_m": self._hold_reacquire_depth,
                "hold_reacquire_pending": self._hold_reacquire_since > 0.0,
                "mount_confirmed": self._mount_confirmed,
                "feedback_ready": self._feedback is not None,
                "target_ready": (
                    self._search_target_ready()
                    if self._servo_mode == "eye_in_hand"
                    else (
                        self._target is not None
                        and time.monotonic() - self._target_time
                        <= self._target_timeout
                    )
                ),
                "hand_distance_range_m": list(self._hand_distance_range),
                "servo_phase": self._servo_phase,
                "typing_plane_ready": self._typing_plane_ready(),
                "typing_plane_updates": self._typing_plane_updates,
                "visible_hand_count": self._visible_hand_count,
                "visual_target_ready": self._visual_target_ready(),
                "predicted_search_ready": self._predicted_search_ready(),
                "using_prediction": self._using_prediction,
                "hand_layout": self._hand_layout,
                "j5_layout_active": self._j5_layout_active,
                "j5_search_mode": self._j5_search_mode,
                "demonstrated_orientation_active": (
                    self._demonstrated_tray_normal is not None
                ),
                "demonstrated_joints_deg": [
                    round(math.degrees(value), 2)
                    for value in self._demonstrated_joints
                ],
                "j5_layout_error_deg": round(self._j5_layout_error_deg, 3),
                "j5_command_delta_deg": round(
                    self._j5_command_delta_deg, 3
                ),
                "j4_image_delta_deg": round(
                    self._j4_image_delta_deg, 3
                ),
                "robust_depth_m": (
                    round(float(self._depth_tracker.value), 4)
                    if self._depth_tracker.initialized
                    else None
                ),
                "robust_depth_velocity_m_s": round(
                    float(self._depth_tracker.velocity), 4
                ),
                "depth_rejected_updates": self._depth_tracker.rejected_count,
                "filtered_hand_camera_m": (
                    [round(float(value), 4) for value in self._last_filtered_hand]
                    if self._last_filtered_hand is not None
                    else None
                ),
                "filtered_hand_distance_m": (
                    round(float(np.linalg.norm(self._last_filtered_hand)), 4)
                    if self._last_filtered_hand is not None
                    else None
                ),
                "filtered_hand_base_m": (
                    [
                        round(float(value), 4)
                        for value in self._last_filtered_hand_base
                    ]
                    if self._last_filtered_hand_base is not None
                    else None
                ),
                "distance_action": self._last_distance_action,
                "cartesian_step_base_m": (
                    [round(float(value), 5) for value in self._last_position_error]
                    if self._last_position_error is not None
                    else None
                ),
                "ik_joint_step_deg": (
                    [
                        round(math.degrees(float(value)), 4)
                        for value in self._last_joint_delta
                    ]
                    if self._last_joint_delta is not None
                    else None
                ),
                "visual_center_error_deg": round(
                    self._last_gaze_error_deg, 3
                ),
                "tracking_center_vertical_offset_deg": round(
                    math.degrees(self._tracking_vertical_offset), 3
                ),
                "tracking_vertical_translation_sign": (
                    self._tracking_vertical_translation_sign
                ),
                "occlusion_downward_step_m": self._occlusion_downward_step,
                "tracking_horizontal_gain": self._tracking_horizontal_gain,
                "single_hand_horizontal_gain": (
                    self._single_hand_horizontal_gain
                ),
                "keyboard_tilt_deg": round(
                    self._last_keyboard_tilt_deg, 3
                ),
                "tray_reference_error_deg": round(
                    self._last_tray_reference_error_deg, 3
                ),
                "control_rate_hz": self._rate,
                "kalman_velocity_m_s": [
                    round(float(value), 4) for value in self._hand_filter.state[3:]
                ],
                "kalman_innovation_m": round(
                    float(self._hand_filter.last_innovation_norm), 4
                ),
                "kalman_gated_updates": self._hand_filter.gated_update_count,
                "prediction_lead_sec": self._prediction_lead,
                "camera_workspace": {
                    "minimum_radial_m": self._workspace[0],
                    "minimum_z_m": self._workspace[1],
                    "maximum_z_m": self._workspace[2],
                    "below_base_minimum_radial_m": self._workspace[3],
                },
                "auto_start_tracking": self._auto_start,
                "startup_nudge_active": self._nudge_active,
                "startup_nudge_done": self._nudge_done,
                "j6_lock_deg": (
                    round(math.degrees(self._j6_lock), 4)
                    if self._j6_lock is not None
                    else None
                ),
                "arm_error": self._arm_error,
                "fault": self._fault,
            }
        )
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianHandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
