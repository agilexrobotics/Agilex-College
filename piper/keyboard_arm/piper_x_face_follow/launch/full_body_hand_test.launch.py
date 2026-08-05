"""One-command real PIPER-X body/hand tracking test bringup.

The launch starts the camera, arm driver, PIPER-X TF, MediaPipe perception,
J6-locked image servo and RViz. The arm and application tracking remain
disabled until their two services are explicitly called.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_port = LaunchConfiguration("can_port")
    camera_launch = PathJoinSubstitution(
        [FindPackageShare("orbbec_camera"), "launch", "dabai.launch.py"]
    )
    tracking_launch = PathJoinSubstitution(
        [
            FindPackageShare("piper_x_face_follow"),
            "launch",
            "body_hand_tracking.launch.py",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("j1_gain_rad_per_m", default_value="-0.70"),
            DeclareLaunchArgument("j1_max_offset_deg", default_value="1.0"),
            DeclareLaunchArgument("max_speed_deg_s", default_value="0.5"),
            DeclareLaunchArgument("driver_speed_percent", default_value="2"),
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
                package="piper",
                executable="piper_single_ctrl",
                name="piper_ctrl_single_node",
                output="screen",
                parameters=[
                    {
                        "can_port": can_port,
                        "auto_enable": False,
                        "gripper_exist": False,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(tracking_launch),
                launch_arguments={
                    "actual_motion": "true",
                    "tf_calibrated": "false",
                    "publish_robot_tf": "true",
                    "publish_camera_mount_tf": "false",
                    "show_image": "false",
                    "show_rviz": LaunchConfiguration("show_rviz"),
                    "j1_gain_rad_per_m": LaunchConfiguration("j1_gain_rad_per_m"),
                    "j1_max_offset_deg": LaunchConfiguration("j1_max_offset_deg"),
                    "j23_max_offset_deg": "0.0",
                    "max_speed_deg_s": LaunchConfiguration("max_speed_deg_s"),
                    "driver_speed_percent": LaunchConfiguration(
                        "driver_speed_percent"
                    ),
                }.items(),
            ),
        ]
    )
