from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("piper_x_face_follow"), "config", "face_follow.yaml"]
    )
    urdf = PathJoinSubstitution(
        [
            FindPackageShare("piper_x_description"),
            "urdf",
            "piper_x_description_no_gripper.urdf",
        ]
    )
    actual_motion = LaunchConfiguration("actual_motion")
    tf_calibrated = LaunchConfiguration("tf_calibrated")
    show_image = LaunchConfiguration("show_image")
    show_rviz = LaunchConfiguration("show_rviz")
    publish_robot_tf = LaunchConfiguration("publish_robot_tf")
    publish_mount_tf = LaunchConfiguration("publish_camera_mount_tf")
    return LaunchDescription(
        [
            DeclareLaunchArgument("actual_motion", default_value="false"),
            DeclareLaunchArgument(
                "tf_calibrated",
                default_value="false",
                description="Explicit assertion that link6->camera_link was measured",
            ),
            DeclareLaunchArgument("show_image", default_value="true"),
            DeclareLaunchArgument("show_rviz", default_value="false"),
            DeclareLaunchArgument("publish_robot_tf", default_value="true"),
            DeclareLaunchArgument(
                "publish_camera_mount_tf",
                default_value="false",
                description="Publish link6->camera_link from the mount arguments",
            ),
            DeclareLaunchArgument("mount_x", default_value="0.0"),
            DeclareLaunchArgument("mount_y", default_value="0.0"),
            DeclareLaunchArgument("mount_z", default_value="0.0"),
            DeclareLaunchArgument("mount_roll", default_value="0.0"),
            DeclareLaunchArgument("mount_pitch", default_value="0.0"),
            DeclareLaunchArgument("mount_yaw", default_value="0.0"),
            DeclareLaunchArgument("j1_max_offset_deg", default_value="1.0"),
            DeclareLaunchArgument("j23_max_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("max_speed_deg_s", default_value="0.5"),
            DeclareLaunchArgument("driver_speed_percent", default_value="2"),
            DeclareLaunchArgument("j1_gain_rad_per_m", default_value="-0.70"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="piper_x_robot_state_publisher",
                parameters=[{"robot_description": ParameterValue(Command(["xacro ", urdf]), value_type=str)}],
                remappings=[("/joint_states", "/joint_states_single")],
                condition=IfCondition(publish_robot_tf),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_mount_tf",
                arguments=[
                    "--x", LaunchConfiguration("mount_x"),
                    "--y", LaunchConfiguration("mount_y"),
                    "--z", LaunchConfiguration("mount_z"),
                    "--roll", LaunchConfiguration("mount_roll"),
                    "--pitch", LaunchConfiguration("mount_pitch"),
                    "--yaw", LaunchConfiguration("mount_yaw"),
                    "--frame-id", "link6",
                    "--child-frame-id", "camera_link",
                ],
                condition=IfCondition(publish_mount_tf),
            ),
            Node(
                package="piper_x_face_follow",
                executable="mediapipe_body_hands",
                name="mediapipe_body_hands",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="piper_x_face_follow",
                executable="ergonomic_target",
                name="ergonomic_target",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="piper_x_face_follow",
                executable="face_follow_controller",
                name="piper_x_body_hand_controller",
                output="screen",
                parameters=[
                    config,
                    {
                        "tracking_source": "hands",
                        "control_frame_mode": "camera_image_servo",
                        "actual_motion": ParameterValue(actual_motion, value_type=bool),
                        "tf_calibrated": ParameterValue(tf_calibrated, value_type=bool),
                        "j1_max_offset_deg": ParameterValue(
                            LaunchConfiguration("j1_max_offset_deg"), value_type=float
                        ),
                        "j23_max_offset_deg": ParameterValue(
                            LaunchConfiguration("j23_max_offset_deg"), value_type=float
                        ),
                        "max_speed_deg_s": ParameterValue(
                            LaunchConfiguration("max_speed_deg_s"), value_type=float
                        ),
                        "driver_speed_percent": ParameterValue(
                            LaunchConfiguration("driver_speed_percent"), value_type=int
                        ),
                        "j1_gain_rad_per_m": ParameterValue(
                            LaunchConfiguration("j1_gain_rad_per_m"), value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="body_hand_image_view",
                arguments=["/human_tracking/debug_image"],
                condition=IfCondition(show_image),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="body_hand_depth_rviz",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("piper_x_face_follow"),
                            "config",
                            "body_hand_depth.rviz",
                        ]
                    ),
                ],
                condition=IfCondition(show_rviz),
            ),
        ]
    )
