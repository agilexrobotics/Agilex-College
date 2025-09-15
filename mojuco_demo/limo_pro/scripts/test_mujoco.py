#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
import math
from pynput import keyboard
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class LimoControl:
    def __init__(self):
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path('/home/khalillee/sim1_ws/src/limo/limo_description/urdf/test.xml')
        self.data = mujoco.MjData(self.model)
        
        # 控制参数
        self.MAX_SPEED = 10.0  # 最大速度 (rad/s)
        self.LINEAR_VEL = 2.0  # 线性速度 (m/s)
        self.ANGULAR_VEL = 10.0  # 角速度 (rad/s)
        
        # ROS参数
        self.use_ros_control = rospy.get_param('~use_ros_control', True)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.05)
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.13)
        self.base_width = rospy.get_param('~base_width', 0.2)
        
        # 初始化ROS节点
        rospy.init_node('limo_mujoco_control')
        
        if self.use_ros_control:
            self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
            rospy.loginfo("使用ROS cmd_vel控制模式")
        else:
            self.start_keyboard_listener()
            rospy.loginfo("使用键盘控制模式")
        
        # 初始化传感器数据发布
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [
            'front_left_wheel', 
            'front_right_wheel',
            'rear_left_wheel',
            'rear_right_wheel'
        ]
        
        # 控制变量
        self.ros_cmd_vel = None
        self.current_keys = set()
        
    def cmd_vel_callback(self, msg):
        """ROS cmd_vel回调函数"""
        self.ros_cmd_vel = msg
        
    def set_motor_velocity(self, vel_fl, vel_fr, vel_rl, vel_rr):
        """设置四个电机的速度"""
        self.data.ctrl[0] = vel_fl  # 左前轮
        self.data.ctrl[1] = vel_fr  # 右前轮
        self.data.ctrl[2] = vel_rl  # 左后轮
        self.data.ctrl[3] = vel_rr  # 右后轮
        
    def stop(self):
        """停止"""
        self.set_motor_velocity(0, 0, 0, 0)
        
    def differential_drive(self, v, w):
        """
        差速驱动控制
        v: 线速度 (m/s)
        w: 角速度 (rad/s)
        """
        # 计算四个轮子的速度 (rad/s)
        vel_fl = (v - w * (self.wheel_separation + self.base_width)/2) / self.wheel_radius
        vel_fr = (v + w * (self.wheel_separation + self.base_width)/2) / self.wheel_radius
        vel_rl = (v - w * (self.wheel_separation + self.base_width)/2) / self.wheel_radius
        vel_rr = (v + w * (self.wheel_separation + self.base_width)/2) / self.wheel_radius
        
        # 限制最大速度
        vel_fl = np.clip(vel_fl, -self.MAX_SPEED, self.MAX_SPEED)
        vel_fr = np.clip(vel_fr, -self.MAX_SPEED, self.MAX_SPEED)
        vel_rl = np.clip(vel_rl, -self.MAX_SPEED, self.MAX_SPEED)
        vel_rr = np.clip(vel_rr, -self.MAX_SPEED, self.MAX_SPEED)
        
        self.set_motor_velocity(vel_fl, vel_fr, vel_rl, vel_rr)
        
    def publish_joint_states(self):
        """发布关节状态"""
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.header.frame_id = "base_link"
        
        # 从模型获取关节位置
        self.joint_state_msg.position = [
            self.data.joint('front_left_wheel').qpos[0],
            self.data.joint('front_right_wheel').qpos[0],
            self.data.joint('rear_left_wheel').qpos[0],
            self.data.joint('rear_right_wheel').qpos[0]
        ]
        
        # 从传感器获取轮速
        self.joint_state_msg.velocity = [
            self.data.sensor('front_left_wheel_vel_sensor').data[0],
            self.data.sensor('front_right_wheel_vel_sensor').data[0],
            self.data.sensor('rear_left_wheel_vel_sensor').data[0],
            self.data.sensor('rear_right_wheel_vel_sensor').data[0]
        ]
        
        # 发布关节状态
        self.joint_state_pub.publish(self.joint_state_msg)
        
    def on_press(self, key):
        """键盘按下事件"""
        try:
            self.current_keys.add(key.char)
        except AttributeError:
            self.current_keys.add(key)
            
    def on_release(self, key):
        """键盘释放事件"""
        try:
            self.current_keys.remove(key.char)
        except AttributeError:
            try:
                self.current_keys.remove(key)
            except KeyError:
                pass
                
    def handle_keyboard_control(self):
        """处理键盘控制"""
        linear = 0.0
        angular = 0.0
        
        if 'i' in self.current_keys or keyboard.Key.up in self.current_keys:
            linear += self.LINEAR_VEL
        if ',' in self.current_keys or keyboard.Key.down in self.current_keys:
            linear -= self.LINEAR_VEL
        if 'j' in self.current_keys or keyboard.Key.left in self.current_keys:
            angular += self.ANGULAR_VEL
        if 'l' in self.current_keys or keyboard.Key.right in self.current_keys:
            angular -= self.ANGULAR_VEL
        if ' ' in self.current_keys:  # 空格键停止
            linear = 0.0
            angular = 0.0
            
        self.differential_drive(linear, angular)
        
    def start_keyboard_listener(self):
        """启动键盘监听"""
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        rospy.loginfo("键盘控制已启用: 8/↑: 前进, 2/↓: 后退, 4/←: 左转, 6/→: 右转, 空格: 停止")
        
    def run(self):
        """主运行循环"""
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while not rospy.is_shutdown() and viewer.is_running():
                    step_start = time.time()
                    
                    # 处理控制输入
                    if self.use_ros_control and self.ros_cmd_vel is not None:
                        self.differential_drive(self.ros_cmd_vel.linear.x, self.ros_cmd_vel.angular.z)
                    else:
                        self.handle_keyboard_control()
                    
                    # 步进模拟
                    mujoco.mj_step(self.model, self.data)
                    
                    # 发布传感器数据
                    self.publish_joint_states()
                    
                    # 同步视图
                    viewer.sync()
                    
                    # 控制循环频率
                    time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                        
        except KeyboardInterrupt:
            rospy.loginfo("Simulation stopped by user")
        finally:
            self.stop()
            rospy.loginfo("Final robot state published")

if __name__ == "__main__":
    control = LimoControl()
    control.run()