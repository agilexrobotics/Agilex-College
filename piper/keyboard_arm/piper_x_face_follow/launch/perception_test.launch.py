from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    show_image = LaunchConfiguration("show_image")
    config = PathJoinSubstitution(
        [FindPackageShare("piper_x_face_follow"), "config", "face_follow.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("show_image", default_value="true"),
            Node(
                package="piper_x_face_follow",
                executable="face_center_detector",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                arguments=["/face_landmarks_debug_image"],
                condition=IfCondition(show_image),
            ),
        ]
    )
