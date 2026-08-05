from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    detector = LaunchConfiguration("detector")
    actual_motion = LaunchConfiguration("actual_motion")
    show_image = LaunchConfiguration("show_image")
    j1_max_offset = LaunchConfiguration("j1_max_offset_deg")
    j23_max_offset = LaunchConfiguration("j23_max_offset_deg")
    max_speed = LaunchConfiguration("max_speed_deg_s")
    driver_speed = LaunchConfiguration("driver_speed_percent")
    config = PathJoinSubstitution(
        [FindPackageShare("piper_x_face_follow"), "config", "face_follow.yaml"]
    )
    yolo_launch = PathJoinSubstitution(
        [FindPackageShare("orbbec_face_landmark"), "launch", "orbbec_face_landmark.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector",
                default_value="fallback",
                description="fallback (built-in OpenCV) or yolo (teammate landmark package)",
            ),
            DeclareLaunchArgument(
                "actual_motion",
                default_value="false",
                description="Allow controller to publish real joint commands after service enable",
            ),
            DeclareLaunchArgument("show_image", default_value="true"),
            DeclareLaunchArgument(
                "j1_max_offset_deg",
                default_value="1.0",
                description="Commissioning limit; expand only after direction is verified",
            ),
            DeclareLaunchArgument(
                "j23_max_offset_deg",
                default_value="0.0",
                description="Vertical tracking is disabled for the first hardware test",
            ),
            DeclareLaunchArgument("max_speed_deg_s", default_value="0.5"),
            DeclareLaunchArgument("driver_speed_percent", default_value="2"),
            Node(
                package="piper_x_face_follow",
                executable="face_center_detector",
                name="face_center_detector",
                output="screen",
                parameters=[config],
                condition=IfCondition(PythonExpression(["'", detector, "' == 'fallback'"])),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(yolo_launch),
                condition=IfCondition(PythonExpression(["'", detector, "' == 'yolo'"])),
            ),
            Node(
                package="piper_x_face_follow",
                executable="face_follow_controller",
                name="piper_x_face_follow_controller",
                output="screen",
                parameters=[
                    config,
                    {
                        "actual_motion": ParameterValue(actual_motion, value_type=bool),
                        "j1_max_offset_deg": ParameterValue(j1_max_offset, value_type=float),
                        "j23_max_offset_deg": ParameterValue(j23_max_offset, value_type=float),
                        "max_speed_deg_s": ParameterValue(max_speed, value_type=float),
                        "driver_speed_percent": ParameterValue(driver_speed, value_type=int),
                    },
                ],
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="face_follow_image_view",
                arguments=["/face_landmarks_debug_image"],
                condition=IfCondition(show_image),
            ),
        ]
    )
