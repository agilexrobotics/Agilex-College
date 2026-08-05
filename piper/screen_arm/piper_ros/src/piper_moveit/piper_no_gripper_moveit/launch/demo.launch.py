from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node

joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    parameters=[{"publish_rate": 200}]  # 修改频率
)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)
