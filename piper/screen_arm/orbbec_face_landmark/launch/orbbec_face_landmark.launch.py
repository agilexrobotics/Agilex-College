import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("orbbec_face_landmark"),
        "config",
        "orbbec_face_landmark.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(name="params_file", default_value=default_params_file),
            DeclareLaunchArgument(
                name="model_path",
                default_value=EnvironmentVariable(
                    "SCREEN_FACE_MODEL", default_value="/home/agilex/tests/piper-x-smart-workstation/screen_arm_ws/src/orbbec_face_landmark/config/yolov5n_face.onnx"
                ),
            ),
            Node(
                package="orbbec_face_landmark",
                executable="orbbec_face_landmark_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "model_path": ParameterValue(
                            LaunchConfiguration("model_path"),
                            value_type=str,
                        )
                    },
                ],
            ),
        ]
    )
