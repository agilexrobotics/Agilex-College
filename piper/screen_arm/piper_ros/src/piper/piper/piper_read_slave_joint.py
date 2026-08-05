#!/usr/bin/env python3
# -*-coding:utf8-*-
# This file controls a single robotic arm node and handles the movement of the robotic arm with a gripper.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import argparse
import math
from piper_sdk import *
from piper_sdk import C_PiperInterface
from scipy.spatial.transform import Rotation as R  # For Euler angle to quaternion conversion
from numpy import clip
from builtin_interfaces.msg import Time
from array import array

class PiperRosNode(Node):
    """ROS2 node for the robotic arm"""

    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # ROS parameters
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('gripper_exist', True)
        # self.declare_parameter('gripper_val_mutiple', 2)

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        # self.gripper_val_mutiple = self.get_parameter('gripper_val_mutiple').get_parameter_value().integer_value
        # self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"gripper_exist is {self.gripper_exist}")
        # self.get_logger().info(f"gripper_val_mutiple is {self.gripper_val_mutiple}")
        # Publishers
        self.joint_feedback_pub = self.create_publisher(JointState, 'joint_states', 1)
        # Joint
        self.joint_states_feedback = JointState()
        # Create piper class and open CAN interface
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    def GetEnableFlag(self):
        return self.__enable_flag

    def publish_thread(self):
        """Publish messages from the robotic arm
        """
        rate = self.create_rate(200)  # 200 Hz
        while rclpy.ok():
            if self.piper.isOk():
                self.PublishArmJointAndGripper()
            else:
                self.get_logger().error(f"{self.can_port} is loss")
                self.get_logger().error(f"exit...")
                rclpy.shutdown() 

            rate.sleep()

    def float_to_ros_time(self, t: float) -> Time:
        ros_time = Time()
        ros_time.sec = int(t)
        ros_time.nanosec = int((t - ros_time.sec) * 1e9)
        return ros_time

    def PublishArmJointAndGripper(self):
        # Assign timestamp
        # self.joint_states_feedback.header.stamp = self.get_clock().now().to_msg()
        new_time = max(self.piper.GetArmJointMsgs().time_stamp, self.piper.GetArmHighSpdInfoMsgs().time_stamp)
        # Here, you can set the joint positions to any value you want
        # The raw data obtained is in degrees multiplied by 1000. To convert to radians, divide by 1000, multiply by π/180, and limit to 5 decimal places
        joint_1: float = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_6: float = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        
        vel_1: float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_2: float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_3: float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_4: float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_5: float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_6: float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.effort/1000
        effort_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.effort/1000
        effort_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.effort/1000
        effort_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.effort/1000
        effort_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.effort/1000
        effort_6:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.effort/1000
        if self.gripper_exist:
            gripper_stroke: float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
            gripper_effort:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
            self.joint_states_feedback.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper','joint7','joint8']
            self.joint_states_feedback.position = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, 0.0, gripper_stroke/2, -gripper_stroke/2]
            self.joint_states_feedback.velocity = [vel_1, vel_2, vel_3, vel_4, vel_5, vel_6, 0.0, 0.0, 0.0]
            self.joint_states_feedback.effort = [effort_1, effort_2, effort_3, effort_4, effort_5, effort_6, 0.0, gripper_effort/2, -gripper_effort/2]
            self.joint_states_feedback.header.stamp = self.float_to_ros_time(new_time)
        else:
            self.joint_states_feedback.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            self.joint_states_feedback.position = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
            self.joint_states_feedback.velocity = [vel_1, vel_2, vel_3, vel_4, vel_5, vel_6]
            self.joint_states_feedback.effort = [effort_1, effort_2, effort_3, effort_4, effort_5, effort_6]
            self.joint_states_feedback.header.stamp = self.float_to_ros_time(new_time)
            
        # 发布所有消息
        if any(abs(pos) > 3.5 for pos in self.joint_states_feedback.position):
            self.get_logger().warn("Joint state abnormal: value exceeds ±3.5 rad")
        else:
            self.joint_feedback_pub.publish(self.joint_states_feedback)

def main(args=None):
    rclpy.init(args=args)
    piper_single_node = PiperRosNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()
