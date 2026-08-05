"""Orbbec depth camera plus MediaPipe body/hand 3D perception."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_launch = PathJoinSubstitution(
        [FindPackageShare("orbbec_camera"), "launch", "dabai.launch.py"]
    )
    config = PathJoinSubstitution(
        [FindPackageShare("piper_x_face_follow"), "config", "face_follow.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("enable_ergonomic_target", default_value="true"),
            DeclareLaunchArgument("gesture_hold_sec", default_value="3.0"),
            DeclareLaunchArgument("gesture_neutral_samples", default_value="4"),
            DeclareLaunchArgument(
                "tracking_center_vertical_offset_deg", default_value="-12.0"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "depth_registration": "true",
                    "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
                    "enable_colored_point_cloud": "false",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "enable_ir": "false",
                    "publish_tf": "true",
                }.items(),
            ),
            Node(
                package="piper_x_face_follow",
                executable="mediapipe_body_hands",
                name="mediapipe_body_hands",
                output="screen",
                parameters=[
                    config,
                    {
                        "gesture_hold_sec": ParameterValue(
                            LaunchConfiguration("gesture_hold_sec"),
                            value_type=float,
                        ),
                        "gesture_neutral_samples": ParameterValue(
                            LaunchConfiguration("gesture_neutral_samples"),
                            value_type=int,
                        ),
                        "tracking_center_vertical_offset_deg": ParameterValue(
                            LaunchConfiguration(
                                "tracking_center_vertical_offset_deg"
                            ),
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="piper_x_face_follow",
                executable="ergonomic_target",
                name="ergonomic_target",
                output="screen",
                parameters=[config],
                condition=IfCondition(
                    LaunchConfiguration("enable_ergonomic_target")
                ),
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
                condition=IfCondition(LaunchConfiguration("show_rviz")),
            ),
        ]
    )
