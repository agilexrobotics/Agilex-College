import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            get_package_share_directory("face_tf_2_piper_link6"),
            "config",
            "face_tf_2_piper_link6.yaml",
        ),
        description="face_tf_2_piper_link6 参数文件",
    )
    calibrate_from_link6_arg = DeclareLaunchArgument(
        "calibrate_from_link6",
        default_value="true",
        description="启动时是否用当前 link6 自动标定 xyz 偏移",
    )

    return LaunchDescription(
        [
            config_file_arg,
            calibrate_from_link6_arg,
            Node(
                package="face_tf_2_piper_link6",
                executable="face_tf_2_piper_link6",
                name="face_tf_2_piper_link6",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "calibrate_from_link6": ParameterValue(
                            LaunchConfiguration("calibrate_from_link6"),
                            value_type=bool,
                        )
                    },
                ],
            ),
        ]
    )
