#!/usr/bin/env python3
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from collections import deque
from filterpy.kalman import KalmanFilter

class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.position_window = deque(maxlen=window_size)
        self.orientation_window = deque(maxlen=window_size)
        
    def filter(self, pose_msg):
        # 提取位置和方向
        position = [pose_msg.pose.position.x, 
                   pose_msg.pose.position.y, 
                   pose_msg.pose.position.z]
        orientation = [pose_msg.pose.orientation.x,
                      pose_msg.pose.orientation.y,
                      pose_msg.pose.orientation.z,
                      pose_msg.pose.orientation.w]
        
        # 添加到窗口
        self.position_window.append(position) 
        self.orientation_window.append(orientation)
        
        # 计算平均值
        avg_position = [sum(x)/len(self.position_window) for x in zip(*self.position_window)]
        avg_orientation = [sum(x)/len(self.orientation_window) for x in zip(*self.orientation_window)]
        
        # 创建新的PoseStamped消息
        filtered_pose = PoseStamped()
        filtered_pose.header = pose_msg.header
        filtered_pose.pose.position.x = avg_position[0]
        filtered_pose.pose.position.y = avg_position[1]
        filtered_pose.pose.position.z = avg_position[2]
        filtered_pose.pose.orientation.x = avg_orientation[0]
        filtered_pose.pose.orientation.y = avg_orientation[1]
        filtered_pose.pose.orientation.z = avg_orientation[2]
        filtered_pose.pose.orientation.w = avg_orientation[3]
        
        return filtered_pose

class PoseKalmanFilter:
    def __init__(self):
        # 创建卡尔曼滤波器 (状态: x, y, z, vx, vy, vz, qx, qy, qz, qw)
        self.kf = KalmanFilter(dim_x=10, dim_z=7)
        
        # 初始化状态转移矩阵 (简单模型)
        self.kf.F = np.eye(10)
        dt = 0.1  # 时间间隔
        # 位置和速度关系
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        
        # 测量矩阵
        self.kf.H = np.zeros((7, 10))
        np.fill_diagonal(self.kf.H[:7, :7], 1)
        
        # 协方差矩阵
        self.kf.P *= 1000
        self.kf.R = np.eye(7) * 5  # 测量噪声
        self.kf.Q = np.eye(10) * 0.1  # 过程噪声
        
        self.last_time = None
        
    def filter(self, pose_msg):
        current_time = rospy.Time.now().to_sec()
        
        # 计算时间间隔
        if self.last_time is None:
            dt = 0.1
        else:
            dt = current_time - self.last_time
        self.last_time = current_time
        
        # 更新状态转移矩阵中的时间参数
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt
        
        # 预测
        self.kf.predict()
        
        # 更新测量值
        z = np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w
        ])
        
        self.kf.update(z)
        
        # 创建过滤后的消息
        filtered_pose = PoseStamped()
        filtered_pose.header = pose_msg.header
        filtered_pose.pose.position.x = self.kf.x[0]
        filtered_pose.pose.position.y = self.kf.x[1]
        filtered_pose.pose.position.z = self.kf.x[2]
        filtered_pose.pose.orientation.x = self.kf.x[3]
        filtered_pose.pose.orientation.y = self.kf.x[4]
        filtered_pose.pose.orientation.z = self.kf.x[5]
        filtered_pose.pose.orientation.w = self.kf.x[6]
        
        return filtered_pose

class LowPassFilter:
    def __init__(self, alpha=0.5):
        self.alpha = alpha  # 平滑因子 (0-1)，越小越平滑
        self.filtered_position = None
        self.filtered_orientation = None
        
    def filter(self, pose_msg):
        position = [pose_msg.pose.position.x, 
                   pose_msg.pose.position.y, 
                   pose_msg.pose.position.z]
        orientation = [pose_msg.pose.orientation.x,
                      pose_msg.pose.orientation.y,
                      pose_msg.pose.orientation.z,
                      pose_msg.pose.orientation.w]
        
        # 初始化过滤值
        if self.filtered_position is None:
            self.filtered_position = position
            self.filtered_orientation = orientation
        else:
            # 应用低通滤波
            self.filtered_position = [
                self.alpha * p + (1 - self.alpha) * f
                for p, f in zip(position, self.filtered_position)
            ]
            self.filtered_orientation = [
                self.alpha * o + (1 - self.alpha) * f
                for o, f in zip(orientation, self.filtered_orientation)
            ]
        
        # 创建新的PoseStamped消息
        filtered_pose = PoseStamped()
        filtered_pose.header = pose_msg.header
        filtered_pose.pose.position.x = self.filtered_position[0]
        filtered_pose.pose.position.y = self.filtered_position[1]
        filtered_pose.pose.position.z = self.filtered_position[2]
        filtered_pose.pose.orientation.x = self.filtered_orientation[0]
        filtered_pose.pose.orientation.y = self.filtered_orientation[1]
        filtered_pose.pose.orientation.z = self.filtered_orientation[2]
        filtered_pose.pose.orientation.w = self.filtered_orientation[3]
        
        return filtered_pose

class PoseFilterNode:
    def __init__(self):
        rospy.init_node('pose_filter_node', anonymous=True)
        
        # 参数
        self.input_topic = rospy.get_param('~input_topic', '/target_pose_output')
        self.output_topic = rospy.get_param('~output_topic', '/target_pose')
        self.filter_type = rospy.get_param('~filter_type', 'low_pass')  # low_pass, moving_avg, kalman
        
        # 初始化滤波器
        if self.filter_type == 'moving_avg':
            self.filter = MovingAverageFilter(window_size=5)
        elif self.filter_type == 'kalman':
            self.filter = PoseKalmanFilter()
        else:  # 默认低通滤波
            self.filter = LowPassFilter(alpha=0.5)
        
        # 订阅和发布
        self.pose_sub = rospy.Subscriber(self.input_topic, PoseStamped, self.pose_callback)
        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)
        
    def pose_callback(self, msg):
        filtered_pose = self.filter.filter(msg)
        self.pose_pub.publish(filtered_pose)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PoseFilterNode()
    node.run()