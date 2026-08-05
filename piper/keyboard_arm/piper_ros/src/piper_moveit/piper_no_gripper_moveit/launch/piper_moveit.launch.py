from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit").to_moveit_configs()

    ld = LaunchDescription()

    # 启动move_group
    my_generate_move_group_launch(ld, moveit_config)
    # 启动rviz
    my_generate_moveit_rviz_launch(ld, moveit_config)

    return ld


def my_generate_move_group_launch(ld, moveit_config):

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]
    move_group_params.append({"use_sim_time": True})

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": ":0"},
    )
    return ld

def my_generate_moveit_rviz_launch(ld, moveit_config):
    """Launch file for rviz"""

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]
    rviz_parameters.append({"use_sim_time": True})

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld
