import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            get_package_share_directory("piper_tf_control"),
            "config",
            "piper_tf_control.yaml",
        ),
        description="piper_tf_control 参数文件",
    )

    return LaunchDescription(
        [
            config_file_arg,
            Node(
                package="piper_tf_control",
                executable="piper_tf_control",
                name="piper_tf_control",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
