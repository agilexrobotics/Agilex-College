import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.event_handlers import OnProcessExit

import xacro

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'piper'
    package_name = 'piper_description'
    urdf_name = "piper_no_gripper_description_gazebo.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')


    # 因为 urdf文件中有一句 $(find mybot) 需要用xacro进行编译一下才行
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    params = {'robot_description': remove_comments(doc.toxml())}

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容
    # 并且会订阅 /joint_states 话题，获取关节的数据，然后发布tf和tf_static话题.
    # 这些节点、话题的名称可不可以自定义？
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )

    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], output='screen')

    # gazebo在加载urdf时，根据urdf的设定，会启动一个joint_states节点?
    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 路径执行控制器，也就是那个action？
    # 系统是如何知道有my_group_controller这个控制器的存在？
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'arm_controller'],
        output='screen'
        )

    # 用下面这两个估计是想控制好各个节点的启动顺序
    # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller？
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller？
    # moveit是怎么和gazebo这里提供的action连接起来的？？
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )

    ld = LaunchDescription()

    ld.add_action(close_evt1)
    ld.add_action(close_evt2)

    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return ld
