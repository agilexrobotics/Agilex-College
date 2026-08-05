"""PIPER-X J1-J5 Cartesian hand following with immutable J6."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf = PathJoinSubstitution(
        [
            FindPackageShare("piper_x_description"),
            "urdf",
            "piper_x_description_no_gripper.urdf",
        ]
    )
    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument("actual_motion", default_value="false"),
            DeclareLaunchArgument("servo_mode", default_value="ergonomic_base"),
            DeclareLaunchArgument("control_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("target_timeout_sec", default_value="0.8"),
            DeclareLaunchArgument("auto_enable", default_value="false"),
            DeclareLaunchArgument("auto_start_tracking", default_value="false"),
            DeclareLaunchArgument("require_start_gesture", default_value="false"),
            DeclareLaunchArgument("home_tolerance_deg", default_value="0.6"),
            DeclareLaunchArgument(
                "home_return_speed_scale", default_value="1.0"
            ),
            DeclareLaunchArgument("settle_hold_enabled", default_value="false"),
            DeclareLaunchArgument(
                "settle_center_error_deg", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "settle_hold_time_sec", default_value="0.7"
            ),
            DeclareLaunchArgument(
                "settle_dropout_grace_sec", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "settle_orientation_error_deg", default_value="6.0"
            ),
            DeclareLaunchArgument(
                "hold_reacquire_xy_m", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "hold_reacquire_depth_m", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "hold_reacquire_time_sec", default_value="0.5"
            ),
            DeclareLaunchArgument("hand_loss_home_sec", default_value="0.0"),
            DeclareLaunchArgument("startup_nudge_deg", default_value="0.0"),
            DeclareLaunchArgument("mount_confirmed", default_value="false"),
            DeclareLaunchArgument("publish_camera_mount_tf", default_value="true"),
            DeclareLaunchArgument("mount_x", default_value="0.0"),
            DeclareLaunchArgument("mount_y", default_value="0.0"),
            DeclareLaunchArgument("mount_z", default_value="0.0"),
            DeclareLaunchArgument("mount_roll", default_value="0.0"),
            DeclareLaunchArgument("mount_pitch", default_value="0.0"),
            DeclareLaunchArgument("mount_yaw", default_value="0.0"),
            DeclareLaunchArgument("tcp_x", default_value="0.0"),
            DeclareLaunchArgument("tcp_y", default_value="0.0"),
            DeclareLaunchArgument("tcp_z", default_value="0.0"),
            DeclareLaunchArgument("max_joint_offset_deg", default_value="2.0"),
            DeclareLaunchArgument("max_joint1_offset_deg", default_value="-1.0"),
            DeclareLaunchArgument("max_joint2_offset_deg", default_value="-1.0"),
            DeclareLaunchArgument("max_joint3_offset_deg", default_value="-1.0"),
            DeclareLaunchArgument("max_joint4_offset_deg", default_value="-1.0"),
            DeclareLaunchArgument("max_joint5_offset_deg", default_value="-1.0"),
            DeclareLaunchArgument("max_joint_speed_deg_s", default_value="0.5"),
            DeclareLaunchArgument(
                "j5_max_joint_speed_deg_s", default_value="4.0"
            ),
            DeclareLaunchArgument(
                "j5_max_joint_accel_deg_s2", default_value="40.0"
            ),
            DeclareLaunchArgument("max_joint_accel_deg_s2", default_value="10.0"),
            DeclareLaunchArgument("driver_speed_percent", default_value="2"),
            DeclareLaunchArgument("max_cartesian_step_m", default_value="0.003"),
            DeclareLaunchArgument("max_target_delta_m", default_value="0.08"),
            DeclareLaunchArgument("minimum_camera_radial_m", default_value="0.18"),
            DeclareLaunchArgument(
                "below_base_minimum_radial_m", default_value="0.18"
            ),
            DeclareLaunchArgument("minimum_camera_z_m", default_value="-0.35"),
            DeclareLaunchArgument("maximum_camera_z_m", default_value="0.75"),
            DeclareLaunchArgument(
                "orientation_weight_m_per_rad", default_value="0.15"
            ),
            DeclareLaunchArgument("damping", default_value="0.04"),
            DeclareLaunchArgument("desired_hand_x_m", default_value="0.0"),
            DeclareLaunchArgument("desired_hand_y_m", default_value="0.0"),
            DeclareLaunchArgument("desired_hand_z_m", default_value="0.50"),
            DeclareLaunchArgument("minimum_hand_distance_m", default_value="0.25"),
            DeclareLaunchArgument("maximum_hand_distance_m", default_value="0.35"),
            DeclareLaunchArgument("hand_xy_deadband_m", default_value="0.01"),
            DeclareLaunchArgument("hand_z_deadband_m", default_value="0.02"),
            DeclareLaunchArgument("kalman_process_noise", default_value="1.0"),
            DeclareLaunchArgument(
                "kalman_measurement_noise", default_value="0.0025"
            ),
            DeclareLaunchArgument("kalman_innovation_gate_m", default_value="0.20"),
            DeclareLaunchArgument("kalman_max_velocity_m_s", default_value="1.5"),
            DeclareLaunchArgument("prediction_lead_sec", default_value="0.06"),
            DeclareLaunchArgument("depth_timeout_sec", default_value="0.55"),
            DeclareLaunchArgument("depth_jump_gate_m", default_value="0.12"),
            DeclareLaunchArgument("depth_stable_updates", default_value="3"),
            DeclareLaunchArgument(
                "distance_reacquire_margin_m", default_value="0.04"
            ),
            DeclareLaunchArgument("align_xy_gain", default_value="0.65"),
            DeclareLaunchArgument("typing_plane_timeout_sec", default_value="0.55"),
            DeclareLaunchArgument(
                "typing_plane_stable_updates", default_value="4"
            ),
            DeclareLaunchArgument(
                "typing_plane_filter_alpha", default_value="0.22"
            ),
            DeclareLaunchArgument("keyboard_max_tilt_deg", default_value="25.0"),
            DeclareLaunchArgument(
                "orientation_error_limit_deg", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "visual_target_timeout_sec", default_value="0.45"
            ),
            DeclareLaunchArgument("visual_filter_alpha", default_value="0.45"),
            DeclareLaunchArgument(
                "tracking_center_vertical_offset_deg", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "tracking_vertical_translation_sign", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "occlusion_downward_step_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "tracking_horizontal_gain", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "single_hand_horizontal_gain", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "visual_center_deadband_deg", default_value="4.0"
            ),
            DeclareLaunchArgument(
                "visual_center_stable_updates", default_value="3"
            ),
            DeclareLaunchArgument(
                "search_prediction_timeout_sec", default_value="0.80"
            ),
            DeclareLaunchArgument(
                "single_hand_retreat_m", default_value="0.035"
            ),
            DeclareLaunchArgument("j4_image_gain", default_value="0.25"),
            DeclareLaunchArgument("j5_image_gain", default_value="0.45"),
            DeclareLaunchArgument("j4_image_sign", default_value="1.0"),
            DeclareLaunchArgument("j5_image_sign", default_value="-1.0"),
            DeclareLaunchArgument(
                "j4_low_head_allowance_deg", default_value="6.0"
            ),
            DeclareLaunchArgument("wrist_level_gain", default_value="0.80"),
            DeclareLaunchArgument(
                "wrist_level_error_limit_deg", default_value="20.0"
            ),
            DeclareLaunchArgument(
                "hand_layout_timeout_sec", default_value="0.45"
            ),
            DeclareLaunchArgument("j5_layout_gain", default_value="0.20"),
            DeclareLaunchArgument("j5_layout_sign", default_value="1.0"),
            DeclareLaunchArgument(
                "j5_layout_deadband_deg", default_value="2.5"
            ),
            DeclareLaunchArgument(
                "j5_camera_right_stop_deg", default_value="52.0"
            ),
            DeclareLaunchArgument(
                "j5_search_right_limit_deg", default_value="60.0"
            ),
            DeclareLaunchArgument(
                "j5_stop_tolerance_deg", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "j5_layout_step_limit_deg", default_value="12.0"
            ),
            DeclareLaunchArgument(
                "single_hand_j5_search_sign", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "single_hand_j5_search_step_deg", default_value="6.0"
            ),
            DeclareLaunchArgument(
                "single_hand_j5_search_limit_deg", default_value="20.0"
            ),
            DeclareLaunchArgument(
                "single_hand_j5_reverse_limit_deg", default_value="10.0"
            ),
            DeclareLaunchArgument(
                "single_hand_j5_start_delay_sec", default_value="0.35"
            ),
            DeclareLaunchArgument(
                "use_demonstrated_orientation", default_value="false"
            ),
            DeclareLaunchArgument(
                "demonstrated_joint1_deg", default_value="48.5"
            ),
            DeclareLaunchArgument(
                "demonstrated_joint2_deg", default_value="158.0"
            ),
            DeclareLaunchArgument(
                "demonstrated_joint3_deg", default_value="-56.2"
            ),
            DeclareLaunchArgument(
                "demonstrated_joint4_deg", default_value="-83.7"
            ),
            DeclareLaunchArgument(
                "demonstrated_joint5_deg", default_value="48.8"
            ),
            DeclareLaunchArgument("nullspace_posture_gain", default_value="0.05"),
            Node(
                package="piper",
                executable="piper_single_ctrl",
                name="piper_ctrl_single_node",
                output="screen",
                parameters=[
                    {
                        "can_port": LaunchConfiguration("can_port"),
                        "auto_enable": ParameterValue(
                            LaunchConfiguration("auto_enable"), value_type=bool
                        ),
                        "gripper_exist": False,
                        # Ctrl-C closes ROS/CAN but deliberately does not send
                        # DisableArm, so the enabled motors retain the last pose.
                        "disable_on_shutdown": False,
                        "log_joint_commands": False,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="piper_x_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", "/joint_states_single")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_mount_tf",
                arguments=[
                    "--x",
                    LaunchConfiguration("mount_x"),
                    "--y",
                    LaunchConfiguration("mount_y"),
                    "--z",
                    LaunchConfiguration("mount_z"),
                    "--roll",
                    LaunchConfiguration("mount_roll"),
                    "--pitch",
                    LaunchConfiguration("mount_pitch"),
                    "--yaw",
                    LaunchConfiguration("mount_yaw"),
                    "--frame-id",
                    "link6",
                    "--child-frame-id",
                    "camera_link",
                ],
                condition=IfCondition(LaunchConfiguration("publish_camera_mount_tf")),
            ),
            Node(
                package="piper_x_face_follow",
                executable="cartesian_hand_controller",
                name="piper_x_cartesian_hand_controller",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "actual_motion": ParameterValue(
                            LaunchConfiguration("actual_motion"), value_type=bool
                        ),
                        "servo_mode": LaunchConfiguration("servo_mode"),
                        "control_rate_hz": ParameterValue(
                            LaunchConfiguration("control_rate_hz"),
                            value_type=float,
                        ),
                        "target_timeout_sec": ParameterValue(
                            LaunchConfiguration("target_timeout_sec"),
                            value_type=float,
                        ),
                        "mount_confirmed": ParameterValue(
                            LaunchConfiguration("mount_confirmed"), value_type=bool
                        ),
                        "auto_start_tracking": ParameterValue(
                            LaunchConfiguration("auto_start_tracking"),
                            value_type=bool,
                        ),
                        "require_start_gesture": ParameterValue(
                            LaunchConfiguration("require_start_gesture"),
                            value_type=bool,
                        ),
                        "home_tolerance_deg": ParameterValue(
                            LaunchConfiguration("home_tolerance_deg"),
                            value_type=float,
                        ),
                        "home_return_speed_scale": ParameterValue(
                            LaunchConfiguration("home_return_speed_scale"),
                            value_type=float,
                        ),
                        "settle_hold_enabled": ParameterValue(
                            LaunchConfiguration("settle_hold_enabled"),
                            value_type=bool,
                        ),
                        "settle_center_error_deg": ParameterValue(
                            LaunchConfiguration("settle_center_error_deg"),
                            value_type=float,
                        ),
                        "settle_hold_time_sec": ParameterValue(
                            LaunchConfiguration("settle_hold_time_sec"),
                            value_type=float,
                        ),
                        "settle_dropout_grace_sec": ParameterValue(
                            LaunchConfiguration("settle_dropout_grace_sec"),
                            value_type=float,
                        ),
                        "settle_orientation_error_deg": ParameterValue(
                            LaunchConfiguration(
                                "settle_orientation_error_deg"
                            ),
                            value_type=float,
                        ),
                        "hold_reacquire_xy_m": ParameterValue(
                            LaunchConfiguration("hold_reacquire_xy_m"),
                            value_type=float,
                        ),
                        "hold_reacquire_depth_m": ParameterValue(
                            LaunchConfiguration("hold_reacquire_depth_m"),
                            value_type=float,
                        ),
                        "hold_reacquire_time_sec": ParameterValue(
                            LaunchConfiguration("hold_reacquire_time_sec"),
                            value_type=float,
                        ),
                        "hand_loss_home_sec": ParameterValue(
                            LaunchConfiguration("hand_loss_home_sec"),
                            value_type=float,
                        ),
                        "startup_nudge_deg": ParameterValue(
                            LaunchConfiguration("startup_nudge_deg"),
                            value_type=float,
                        ),
                        "tcp_x": ParameterValue(
                            LaunchConfiguration("tcp_x"), value_type=float
                        ),
                        "tcp_y": ParameterValue(
                            LaunchConfiguration("tcp_y"), value_type=float
                        ),
                        "tcp_z": ParameterValue(
                            LaunchConfiguration("tcp_z"), value_type=float
                        ),
                        "max_joint_offset_deg": ParameterValue(
                            LaunchConfiguration("max_joint_offset_deg"),
                            value_type=float,
                        ),
                        **{
                            f"max_joint{index}_offset_deg": ParameterValue(
                                LaunchConfiguration(
                                    f"max_joint{index}_offset_deg"
                                ),
                                value_type=float,
                            )
                            for index in range(1, 6)
                        },
                        "max_joint_speed_deg_s": ParameterValue(
                            LaunchConfiguration("max_joint_speed_deg_s"),
                            value_type=float,
                        ),
                        "j5_max_joint_speed_deg_s": ParameterValue(
                            LaunchConfiguration("j5_max_joint_speed_deg_s"),
                            value_type=float,
                        ),
                        "j5_max_joint_accel_deg_s2": ParameterValue(
                            LaunchConfiguration("j5_max_joint_accel_deg_s2"),
                            value_type=float,
                        ),
                        "max_joint_accel_deg_s2": ParameterValue(
                            LaunchConfiguration("max_joint_accel_deg_s2"),
                            value_type=float,
                        ),
                        "driver_speed_percent": ParameterValue(
                            LaunchConfiguration("driver_speed_percent"),
                            value_type=int,
                        ),
                        "max_cartesian_step_m": ParameterValue(
                            LaunchConfiguration("max_cartesian_step_m"),
                            value_type=float,
                        ),
                        "max_target_delta_m": ParameterValue(
                            LaunchConfiguration("max_target_delta_m"),
                            value_type=float,
                        ),
                        "minimum_camera_radial_m": ParameterValue(
                            LaunchConfiguration("minimum_camera_radial_m"),
                            value_type=float,
                        ),
                        "below_base_minimum_radial_m": ParameterValue(
                            LaunchConfiguration("below_base_minimum_radial_m"),
                            value_type=float,
                        ),
                        "minimum_camera_z_m": ParameterValue(
                            LaunchConfiguration("minimum_camera_z_m"),
                            value_type=float,
                        ),
                        "maximum_camera_z_m": ParameterValue(
                            LaunchConfiguration("maximum_camera_z_m"),
                            value_type=float,
                        ),
                        "orientation_weight_m_per_rad": ParameterValue(
                            LaunchConfiguration("orientation_weight_m_per_rad"),
                            value_type=float,
                        ),
                        "damping": ParameterValue(
                            LaunchConfiguration("damping"),
                            value_type=float,
                        ),
                        "desired_hand_x_m": ParameterValue(
                            LaunchConfiguration("desired_hand_x_m"),
                            value_type=float,
                        ),
                        "desired_hand_y_m": ParameterValue(
                            LaunchConfiguration("desired_hand_y_m"),
                            value_type=float,
                        ),
                        "desired_hand_z_m": ParameterValue(
                            LaunchConfiguration("desired_hand_z_m"),
                            value_type=float,
                        ),
                        "minimum_hand_distance_m": ParameterValue(
                            LaunchConfiguration("minimum_hand_distance_m"),
                            value_type=float,
                        ),
                        "maximum_hand_distance_m": ParameterValue(
                            LaunchConfiguration("maximum_hand_distance_m"),
                            value_type=float,
                        ),
                        "hand_xy_deadband_m": ParameterValue(
                            LaunchConfiguration("hand_xy_deadband_m"),
                            value_type=float,
                        ),
                        "hand_z_deadband_m": ParameterValue(
                            LaunchConfiguration("hand_z_deadband_m"),
                            value_type=float,
                        ),
                        "kalman_process_noise": ParameterValue(
                            LaunchConfiguration("kalman_process_noise"),
                            value_type=float,
                        ),
                        "kalman_measurement_noise": ParameterValue(
                            LaunchConfiguration("kalman_measurement_noise"),
                            value_type=float,
                        ),
                        "kalman_innovation_gate_m": ParameterValue(
                            LaunchConfiguration("kalman_innovation_gate_m"),
                            value_type=float,
                        ),
                        "kalman_max_velocity_m_s": ParameterValue(
                            LaunchConfiguration("kalman_max_velocity_m_s"),
                            value_type=float,
                        ),
                        "prediction_lead_sec": ParameterValue(
                            LaunchConfiguration("prediction_lead_sec"),
                            value_type=float,
                        ),
                        "depth_timeout_sec": ParameterValue(
                            LaunchConfiguration("depth_timeout_sec"),
                            value_type=float,
                        ),
                        "depth_jump_gate_m": ParameterValue(
                            LaunchConfiguration("depth_jump_gate_m"),
                            value_type=float,
                        ),
                        "depth_stable_updates": ParameterValue(
                            LaunchConfiguration("depth_stable_updates"),
                            value_type=int,
                        ),
                        "distance_reacquire_margin_m": ParameterValue(
                            LaunchConfiguration("distance_reacquire_margin_m"),
                            value_type=float,
                        ),
                        "align_xy_gain": ParameterValue(
                            LaunchConfiguration("align_xy_gain"),
                            value_type=float,
                        ),
                        "typing_plane_timeout_sec": ParameterValue(
                            LaunchConfiguration("typing_plane_timeout_sec"),
                            value_type=float,
                        ),
                        "typing_plane_stable_updates": ParameterValue(
                            LaunchConfiguration("typing_plane_stable_updates"),
                            value_type=int,
                        ),
                        "typing_plane_filter_alpha": ParameterValue(
                            LaunchConfiguration("typing_plane_filter_alpha"),
                            value_type=float,
                        ),
                        "keyboard_max_tilt_deg": ParameterValue(
                            LaunchConfiguration("keyboard_max_tilt_deg"),
                            value_type=float,
                        ),
                        "orientation_error_limit_deg": ParameterValue(
                            LaunchConfiguration("orientation_error_limit_deg"),
                            value_type=float,
                        ),
                        "visual_target_timeout_sec": ParameterValue(
                            LaunchConfiguration("visual_target_timeout_sec"),
                            value_type=float,
                        ),
                        "visual_filter_alpha": ParameterValue(
                            LaunchConfiguration("visual_filter_alpha"),
                            value_type=float,
                        ),
                        "tracking_center_vertical_offset_deg": ParameterValue(
                            LaunchConfiguration(
                                "tracking_center_vertical_offset_deg"
                            ),
                            value_type=float,
                        ),
                        "tracking_vertical_translation_sign": ParameterValue(
                            LaunchConfiguration(
                                "tracking_vertical_translation_sign"
                            ),
                            value_type=float,
                        ),
                        "occlusion_downward_step_m": ParameterValue(
                            LaunchConfiguration("occlusion_downward_step_m"),
                            value_type=float,
                        ),
                        "tracking_horizontal_gain": ParameterValue(
                            LaunchConfiguration("tracking_horizontal_gain"),
                            value_type=float,
                        ),
                        "single_hand_horizontal_gain": ParameterValue(
                            LaunchConfiguration("single_hand_horizontal_gain"),
                            value_type=float,
                        ),
                        "visual_center_deadband_deg": ParameterValue(
                            LaunchConfiguration("visual_center_deadband_deg"),
                            value_type=float,
                        ),
                        "visual_center_stable_updates": ParameterValue(
                            LaunchConfiguration("visual_center_stable_updates"),
                            value_type=int,
                        ),
                        "search_prediction_timeout_sec": ParameterValue(
                            LaunchConfiguration("search_prediction_timeout_sec"),
                            value_type=float,
                        ),
                        "single_hand_retreat_m": ParameterValue(
                            LaunchConfiguration("single_hand_retreat_m"),
                            value_type=float,
                        ),
                        "j4_image_gain": ParameterValue(
                            LaunchConfiguration("j4_image_gain"),
                            value_type=float,
                        ),
                        "j5_image_gain": ParameterValue(
                            LaunchConfiguration("j5_image_gain"),
                            value_type=float,
                        ),
                        "j4_image_sign": ParameterValue(
                            LaunchConfiguration("j4_image_sign"),
                            value_type=float,
                        ),
                        "j5_image_sign": ParameterValue(
                            LaunchConfiguration("j5_image_sign"),
                            value_type=float,
                        ),
                        "j4_low_head_allowance_deg": ParameterValue(
                            LaunchConfiguration("j4_low_head_allowance_deg"),
                            value_type=float,
                        ),
                        "wrist_level_gain": ParameterValue(
                            LaunchConfiguration("wrist_level_gain"),
                            value_type=float,
                        ),
                        "wrist_level_error_limit_deg": ParameterValue(
                            LaunchConfiguration("wrist_level_error_limit_deg"),
                            value_type=float,
                        ),
                        "hand_layout_timeout_sec": ParameterValue(
                            LaunchConfiguration("hand_layout_timeout_sec"),
                            value_type=float,
                        ),
                        "j5_layout_gain": ParameterValue(
                            LaunchConfiguration("j5_layout_gain"),
                            value_type=float,
                        ),
                        "j5_layout_sign": ParameterValue(
                            LaunchConfiguration("j5_layout_sign"),
                            value_type=float,
                        ),
                        "j5_layout_deadband_deg": ParameterValue(
                            LaunchConfiguration("j5_layout_deadband_deg"),
                            value_type=float,
                        ),
                        "j5_camera_right_stop_deg": ParameterValue(
                            LaunchConfiguration("j5_camera_right_stop_deg"),
                            value_type=float,
                        ),
                        "j5_search_right_limit_deg": ParameterValue(
                            LaunchConfiguration("j5_search_right_limit_deg"),
                            value_type=float,
                        ),
                        "j5_stop_tolerance_deg": ParameterValue(
                            LaunchConfiguration("j5_stop_tolerance_deg"),
                            value_type=float,
                        ),
                        "j5_layout_step_limit_deg": ParameterValue(
                            LaunchConfiguration("j5_layout_step_limit_deg"),
                            value_type=float,
                        ),
                        "single_hand_j5_search_sign": ParameterValue(
                            LaunchConfiguration("single_hand_j5_search_sign"),
                            value_type=float,
                        ),
                        "single_hand_j5_search_step_deg": ParameterValue(
                            LaunchConfiguration(
                                "single_hand_j5_search_step_deg"
                            ),
                            value_type=float,
                        ),
                        "single_hand_j5_search_limit_deg": ParameterValue(
                            LaunchConfiguration(
                                "single_hand_j5_search_limit_deg"
                            ),
                            value_type=float,
                        ),
                        "single_hand_j5_reverse_limit_deg": ParameterValue(
                            LaunchConfiguration(
                                "single_hand_j5_reverse_limit_deg"
                            ),
                            value_type=float,
                        ),
                        "single_hand_j5_start_delay_sec": ParameterValue(
                            LaunchConfiguration(
                                "single_hand_j5_start_delay_sec"
                            ),
                            value_type=float,
                        ),
                        "use_demonstrated_orientation": ParameterValue(
                            LaunchConfiguration("use_demonstrated_orientation"),
                            value_type=bool,
                        ),
                        **{
                            f"demonstrated_joint{index}_deg": ParameterValue(
                                LaunchConfiguration(
                                    f"demonstrated_joint{index}_deg"
                                ),
                                value_type=float,
                            )
                            for index in range(1, 6)
                        },
                        "nullspace_posture_gain": ParameterValue(
                            LaunchConfiguration("nullspace_posture_gain"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
