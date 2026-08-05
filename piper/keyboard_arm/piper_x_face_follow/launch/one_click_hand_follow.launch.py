"""One-click low-speed real PIPER-X hand-following bringup."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("piper_x_face_follow")
    camera_launch = PathJoinSubstitution(
        [share, "launch", "camera_hand_tracking.launch.py"]
    )
    arm_launch = PathJoinSubstitution(
        [share, "launch", "arm_cartesian_hand_follow.launch.py"]
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "show_rviz": "true",
                    "enable_point_cloud": "true",
                    "enable_ergonomic_target": "false",
                    "gesture_hold_sec": "1.0",
                    "gesture_neutral_samples": "2",
                    "tracking_center_vertical_offset_deg": "-12.0",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(arm_launch),
                launch_arguments={
                    "can_port": "can0",
                    "actual_motion": "true",
                    "servo_mode": "eye_in_hand",
                    "control_rate_hz": "150.0",
                    "target_timeout_sec": "0.55",
                    "auto_enable": "true",
                    "auto_start_tracking": "true",
                    "require_start_gesture": "true",
                    "home_tolerance_deg": "0.6",
                    "home_return_speed_scale": "0.5",
                    "settle_hold_enabled": "true",
                    "settle_center_error_deg": "14.0",
                    "settle_hold_time_sec": "0.5",
                    "settle_dropout_grace_sec": "0.6",
                    "settle_orientation_error_deg": "6.0",
                    "hold_reacquire_xy_m": "0.10",
                    "hold_reacquire_depth_m": "0.10",
                    "hold_reacquire_time_sec": "0.5",
                    "hand_loss_home_sec": "6.0",
                    # Stay completely still until a stable one-finger gesture.
                    "startup_nudge_deg": "0.0",
                    "mount_confirmed": "true",
                    # Physical description supplied by the user: camera is above
                    # J6 and points slightly downward. Translation is a safe
                    # initial estimate and should later be measured from CAD.
                    # Camera is roughly 0.20 m above J6 in the physical startup
                    # pose and points out from the table with a slight downward
                    # tilt. Values are expressed in link6 coordinates.
                    "mount_x": "-0.19504",
                    "mount_y": "-0.02223",
                    "mount_z": "-0.03827",
                    "mount_roll": "1.64437",
                    "mount_pitch": "-1.17497",
                    "mount_yaw": "-1.45575",
                    "tcp_x": "-0.19504",
                    "tcp_y": "-0.02223",
                    "tcp_z": "-0.03827",
                    # The old uniform +/-30 deg envelope prevented the arm from
                    # folding J2/J3 below the table-edge-mounted base.
                    "max_joint_offset_deg": "30.0",
                    "max_joint1_offset_deg": "30.0",
                    "max_joint2_offset_deg": "165.0",
                    "max_joint3_offset_deg": "165.0",
                    # The real teaching path moved J4 from +35 deg to -90 deg.
                    # A 60 deg startup envelope was the reason the endpoint
                    # could not descend while keeping the tray orientation.
                    "max_joint4_offset_deg": "130.0",
                    "max_joint5_offset_deg": "60.0",
                    # 2x the previous profile. The driver percentage remains
                    # at its valid maximum while velocity, acceleration and
                    # Cartesian step scale together.
                    "max_joint_speed_deg_s": "270.0",
                    "j5_max_joint_speed_deg_s": "162.0",
                    "j5_max_joint_accel_deg_s2": "1080.0",
                    "max_joint_accel_deg_s2": "1440.0",
                    "driver_speed_percent": "100",
                    "max_cartesian_step_m": "0.0090",
                    "max_target_delta_m": "0.30",
                    "minimum_camera_radial_m": "0.07",
                    "below_base_minimum_radial_m": "0.18",
                    "minimum_camera_z_m": "-0.35",
                    "maximum_camera_z_m": "0.75",
                    "orientation_weight_m_per_rad": "0.08",
                    "damping": "0.012",
                    "desired_hand_x_m": "0.0",
                    # Search policy requires the visible hand target at the
                    # optical centre before any forward approach.
                    "desired_hand_y_m": "0.0",
                    "desired_hand_z_m": "0.26",
                    "minimum_hand_distance_m": "0.22",
                    "maximum_hand_distance_m": "0.30",
                    "hand_xy_deadband_m": "0.012",
                    "hand_z_deadband_m": "0.008",
                    "kalman_process_noise": "4.0",
                    "kalman_measurement_noise": "0.0008",
                    "kalman_innovation_gate_m": "0.20",
                    "kalman_max_velocity_m_s": "1.5",
                    "prediction_lead_sec": "0.04",
                    "depth_timeout_sec": "0.70",
                    "depth_jump_gate_m": "0.12",
                    "depth_stable_updates": "3",
                    "distance_reacquire_margin_m": "0.04",
                    "align_xy_gain": "0.18",
                    "typing_plane_timeout_sec": "0.60",
                    "typing_plane_stable_updates": "4",
                    "typing_plane_filter_alpha": "0.22",
                    "keyboard_max_tilt_deg": "25.0",
                    "orientation_error_limit_deg": "25.0",
                    "visual_target_timeout_sec": "0.45",
                    "visual_filter_alpha": "0.45",
                    "tracking_center_vertical_offset_deg": "-12.0",
                    "tracking_vertical_translation_sign": "-1.0",
                    "occlusion_downward_step_m": "0.00080",
                    "tracking_horizontal_gain": "0.30",
                    "single_hand_horizontal_gain": "0.05",
                    "visual_center_deadband_deg": "4.0",
                    "visual_center_stable_updates": "3",
                    "search_prediction_timeout_sec": "0.80",
                    "single_hand_retreat_m": "0.035",
                    # Image midpoint errors are translations, never wrist
                    # rotations. These legacy gains are intentionally zero.
                    "j4_image_gain": "0.0",
                    "j5_image_gain": "0.0",
                    "j4_image_sign": "1.0",
                    "j5_image_sign": "-1.0",
                    "j4_low_head_allowance_deg": "6.0",
                    "wrist_level_gain": "0.80",
                    "wrist_level_error_limit_deg": "20.0",
                    "hand_layout_timeout_sec": "0.45",
                    "j5_layout_gain": "0.18",
                    # The latest physical log proves positive J5 moves the
                    # camera left, so right-facing corrections are negative.
                    "j5_layout_sign": "-1.0",
                    "j5_layout_deadband_deg": "6.0",
                    "j5_camera_right_stop_deg": "45.0",
                    "j5_search_right_limit_deg": "60.0",
                    "j5_stop_tolerance_deg": "0.5",
                    "j5_layout_step_limit_deg": "3.0",
                    # When one hand leaves the frame, keep rotating J5 in the
                    # last proven direction (camera-right is positive on this
                    # physical arm) only after a short pair-loss debounce.
                    # Keep this sweep small so detector flicker cannot make
                    # the wrist oscillate.
                    "single_hand_j5_search_sign": "-1.0",
                    "single_hand_j5_search_step_deg": "2.0",
                    "single_hand_j5_search_limit_deg": "6.0",
                    "single_hand_j5_reverse_limit_deg": "4.0",
                    "single_hand_j5_start_delay_sec": "0.35",
                    "use_demonstrated_orientation": "true",
                    "demonstrated_joint1_deg": "48.5",
                    "demonstrated_joint2_deg": "158.0",
                    "demonstrated_joint3_deg": "-56.2",
                    "demonstrated_joint4_deg": "-83.7",
                    "demonstrated_joint5_deg": "48.8",
                    "nullspace_posture_gain": "0.0002",
                }.items(),
            ),
        ]
    )
