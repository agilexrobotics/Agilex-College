#!/usr/bin/env python3
# _*_ coding:utf-8 _*_

import rospy
import cv2
import mediapipe as mp
import math
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Quaternion
from std_msgs.msg import ColorRGBA
import time
import tf2_ros
import tf
import tf.transformations as tf_trans
import sys
import select
import tty
import termios
from std_msgs.msg import Bool, Float64

class GestureRecognizer:
    def __init__(self):
        rospy.init_node('gesture_recognizer', anonymous=True)
        # 初始化 MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

        self.bridge = CvBridge()

        self.count = 0
        
        # 相机参数（初始化为None，将从camera_info获取）
        self.fx = None  # 水平焦距 (像素)
        self.fy = None  # 垂直焦距 (像素)
        self.cx = None  # 主点x (像素)
        self.cy = None  # 主点y (像素)
        self.camera_info_received = False
        
        # 记录机械臂末端的坐标
        self.armEndPos = PoseStamped()
        self.armEndPos.header.frame_id = "base_link"  # 基坐标系原点的坐标系名称
        self.armEndPos.pose.position.x = 0.335391640663147
        self.armEndPos.pose.position.y = -3.91155481338501e-08
        self.armEndPos.pose.position.z = 0.2967674732208252
        self.armEndPos.pose.orientation.x = -1.1353528606150576e-07
        self.armEndPos.pose.orientation.y = 0.953766942024231
        self.armEndPos.pose.orientation.z = 1.811207184232444e-09
        self.armEndPos.pose.orientation.w = 0.3005475699901581

        # 手掌边界框中心坐标
        self.centroid = (0, 0, 0)
        
        # 添加平滑滤波器参数
        self.smoothing_factor = 0.3  # 平滑系数 (0-1)，值越小越平滑
        self.smoothed_centroid = None  # 平滑后的中心坐标
        self.smoothed_rotation = None  # 平滑后的旋转四元数
        
        # 添加历史数据队列用于移动平均
        self.position_history = []
        self.rotation_history = []
        self.history_size = 5  # 历史数据队列大小
        
        # 订阅图像话题
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        # self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.endPosition_sub = rospy.Subscriber("/target_pose", PoseStamped, self.endPostion_callback)
        
        # 订阅相机内参信息
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        
        # 发布带标记的图像
        self.marked_image_pub = rospy.Publisher("/gesture_recognition/marked_image", Image, queue_size=1)
        # 发布手部关键点MarkerArray
        self.marker_pub = rospy.Publisher("/hand_landmarks_markers", MarkerArray, queue_size=1)
        # 发布基坐标系原点
        self.base_frame_pub = rospy.Publisher("/base_frame_origin", PoseStamped, queue_size=1)
        # 发布相对基坐标系原点的相对位置
        self.pos_cmd_pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=10)
        # 夹爪控制发布
        self.gripper_cmd_pub = rospy.Publisher('/gripper_cmd_topic', Float64, queue_size=1)
        self.gripper_cmd = 0  # 夹爪状态
        self.is_end_pose_recieved = False

        # 存储最新的深度图
        self.latest_depth = None
        
        # 手势状态跟踪
        self.last_gesture = None
        self.gesture_start_time = None
        self.base_frame_origin = None  # 存储基坐标系原点 [x, y, z]
        self.base_frame_rotation = None  # 存储基坐标系的旋转（四元数）

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 设置非阻塞输入
        self.old_settings = termios.tcgetattr(sys.stdin)
        self.rate = rospy.Rate(30)  # 30Hz
        self.is_record_arm_endpos = False
        
        rospy.loginfo("Gesture Recognizer initialized")
        rospy.loginfo("Waiting for camera info...")

    def smooth_position(self, new_position):
        """使用指数移动平均和移动平均结合的方法平滑位置"""
        if self.smoothed_centroid is None:
            self.smoothed_centroid = new_position
            return new_position
        
        # 指数移动平均
        smoothed = (
            self.smoothed_centroid[0] * (1 - self.smoothing_factor) + new_position[0] * self.smoothing_factor,
            self.smoothed_centroid[1] * (1 - self.smoothing_factor) + new_position[1] * self.smoothing_factor,
            self.smoothed_centroid[2] * (1 - self.smoothing_factor) + new_position[2] * self.smoothing_factor
        )
        
        # 添加到历史队列
        self.position_history.append(smoothed)
        if len(self.position_history) > self.history_size:
            self.position_history.pop(0)
        
        # 计算移动平均
        if len(self.position_history) > 0:
            avg_x = sum(p[0] for p in self.position_history) / len(self.position_history)
            avg_y = sum(p[1] for p in self.position_history) / len(self.position_history)
            avg_z = sum(p[2] for p in self.position_history) / len(self.position_history)
            self.smoothed_centroid = (avg_x, avg_y, avg_z)
        
        return self.smoothed_centroid
    
    def smooth_rotation(self, new_rotation):
        """平滑旋转四元数 - 改进版本"""
        if self.smoothed_rotation is None:
            self.smoothed_rotation = new_rotation
            self.rotation_history = [new_rotation] * self.history_size  # 用当前值初始化历史
            return new_rotation
        
        # 检查四元数是否有效
        if not self.is_valid_quaternion(new_rotation):
            rospy.logwarn("Invalid quaternion received, using previous value")
            return self.smoothed_rotation
        
        # 添加到历史队列
        self.rotation_history.append(new_rotation)
        if len(self.rotation_history) > self.history_size:
            self.rotation_history.pop(0)
        
        # 使用加权滑动平均和球面线性插值
        if len(self.rotation_history) >= 2:
            # 方法1：加权平均（更重视最近的数据）
            smoothed_quat = self.weighted_quaternion_average(self.rotation_history)
            
            # 方法2：与上一帧进行SLERP插值（更平滑的过渡）
            current_q = np.array([self.smoothed_rotation.x, self.smoothed_rotation.y, 
                                self.smoothed_rotation.z, self.smoothed_rotation.w])
            target_q = np.array([smoothed_quat[0], smoothed_quat[1], 
                                smoothed_quat[2], smoothed_quat[3]])
            
            # 使用SLERP进行平滑插值
            final_quat = self.slerp(current_q, target_q, self.smoothing_factor)
            
            self.smoothed_rotation = Quaternion(
                final_quat[0], final_quat[1], final_quat[2], final_quat[3]
            )
        
        return self.smoothed_rotation

    def is_valid_quaternion(self, quat):
        """检查四元数是否有效"""
        magnitude_sq = (quat.x ** 2 + quat.y ** 2 + quat.z ** 2 + quat.w ** 2)
        return 0.9 < magnitude_sq < 1.1  # 允许一定的数值误差

    def weighted_quaternion_average(self, quaternions):
        """加权四元数平均，更重视最近的数据"""
        n = len(quaternions)
        weights = np.linspace(0.1, 1.0, n)  # 线性权重，最近的最大
        weights = weights / np.sum(weights)  # 归一化权重
        
        # 转换为numpy数组
        quats_array = np.array([[q.x, q.y, q.z, q.w] for q in quaternions])
        
        # 使用特征值方法计算加权平均（更稳定）
        Q = np.zeros((4, 4))
        for i, q in enumerate(quats_array):
            # 确保四元数单位化
            q_normalized = q / np.linalg.norm(q)
            # 构建外积矩阵并加权
            outer_product = np.outer(q_normalized, q_normalized)
            Q += weights[i] * outer_product
        
        # 计算最大特征值对应的特征向量（即平均四元数）
        eigenvalues, eigenvectors = np.linalg.eig(Q)
        max_eigenvalue_index = np.argmax(eigenvalues)
        avg_quat = eigenvectors[:, max_eigenvalue_index]
        
        # 确保四元数在正确的半球
        if avg_quat[3] < 0:
            avg_quat = -avg_quat
        
        return avg_quat / np.linalg.norm(avg_quat)

    def slerp(self, q1, q2, t):
        """球面线性插值"""
        # 归一化四元数
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # 计算点积来确定插值方向
        dot = np.dot(q1, q2)
        
        # 如果点积为负，反转一个四元数以取最短路径
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # 如果四元数非常接近，使用线性插值避免数值问题
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # 计算插值角度
        theta_0 = np.arccos(dot)  # 角度
        theta = theta_0 * t       # 插值角度
        
        # 计算插值四元数
        q3 = q2 - q1 * dot
        q3 = q3 / np.linalg.norm(q3)
        
        return q1 * np.cos(theta) + q3 * np.sin(theta)

    def exponential_moving_average_quaternion(self, current, new, alpha):
        """四元数指数移动平均"""
        # 确保四元数单位化
        current = current / np.linalg.norm(current)
        new = new / np.linalg.norm(new)
        
        # 计算点积来确定插值方向
        dot = np.dot(current, new)
        
        # 如果点积为负，反转新四元数
        if dot < 0.0:
            new = -new
        
        # 使用SLERP进行指数移动平均
        return self.slerp(current, new, alpha)

    def __del__(self):
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        # 非阻塞键盘输入检查
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def publish_gripper_control(self):
        self.gripper_cmd_pub.publish(self.gripper_cmd)

    def camera_info_callback(self, msg):
        """从camera_info消息中获取相机内参"""
        if not self.camera_info_received:
            # 相机内参矩阵K是一个3x3矩阵，按行优先顺序排列
            # K = [fx, 0, cx]
            #     [0, fy, cy]
            #     [0, 0, 1]
            self.fx = msg.K[0]  # 水平焦距
            self.fy = msg.K[4]  # 垂直焦距
            self.cx = msg.K[2]  # 主点x
            self.cy = msg.K[5]  # 主点y
            
            self.camera_info_received = True
            rospy.loginfo(f"Camera parameters received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def pixel_to_3d(self, u, v, depth):
        """将像素坐标+深度值转换为3D坐标(相机坐标系)"""
        # 检查是否已收到相机内参
        if not self.camera_info_received:
            rospy.logwarn_throttle(1.0, "Camera parameters not received yet")
            return None
            
        if depth == 0:
            return None
        
        Z = depth / 1000.0  # 转换为米
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return (X, Y, Z)
    
    def calculate_distance(self, p1, p2):
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
    
    def calculate_distance_new(self, p1, p2):
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

    def calculate_hand_bbox_center(self, hand_landmarks, image_shape):
        """计算手掌边界框的中心点"""
        # 检查是否已收到相机内参
        if not self.camera_info_received:
            return None
            
        # 获取所有关键点的像素坐标
        landmarks_px = []
        for landmark in hand_landmarks.landmark:
            x = int(landmark.x * image_shape[1])
            y = int(landmark.y * image_shape[0])
            landmarks_px.append((x, y))
        
        if not landmarks_px:
            return None
        
        # 计算边界框
        x_coords = [p[0] for p in landmarks_px]
        y_coords = [p[1] for p in landmarks_px]
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # 计算中心点
        center_x = (min_x + max_x) // 2
        center_y = (min_y + max_y) // 2
        
        # 获取中心点的深度
        if (self.latest_depth is not None and 
            0 <= center_y < self.latest_depth.shape[0] and 
            0 <= center_x < self.latest_depth.shape[1]):
            depth = self.latest_depth[center_y, center_x]
            point_3d = self.pixel_to_3d(center_x, center_y, depth)
            return point_3d
        return None

    def calculate_hand_orientation(self, hand_landmarks, landmarks_3d):
        """计算手的旋转方向 - 优化版本"""
        try:
            # 获取关键点索引
            wrist_idx = self.mp_hands.HandLandmark.WRIST
            middle_mcp_idx = self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP
            index_mcp_idx = self.mp_hands.HandLandmark.INDEX_FINGER_MCP
            pinky_mcp_idx = self.mp_hands.HandLandmark.PINKY_MCP
            thumb_cmc_idx = self.mp_hands.HandLandmark.THUMB_CMC
            
            # 获取3D坐标并检查有效性
            wrist = np.array(landmarks_3d[wrist_idx])
            middle_mcp = np.array(landmarks_3d[middle_mcp_idx])
            index_mcp = np.array(landmarks_3d[index_mcp_idx])
            pinky_mcp = np.array(landmarks_3d[pinky_mcp_idx])
            thumb_cmc = np.array(landmarks_3d[thumb_cmc_idx])
            
            # 方法1：使用手掌平面法向量（更稳定）
            # 计算手掌平面的两个向量
            v1 = index_mcp - wrist  # 手腕到食指MCP
            v2 = pinky_mcp - wrist  # 手腕到小指MCP
            
            # 计算手掌法向量（手掌朝前的方向）
            palm_normal = np.cross(v1, v2)
            palm_normal_norm = np.linalg.norm(palm_normal)
            
            # 检查法向量是否有效
            if palm_normal_norm < 1e-6:
                rospy.logwarn("Invalid palm normal, using default orientation")
                return Quaternion(0, 0, 0, 1)
            
            palm_normal = palm_normal / palm_normal_norm
            
            # 方法2：计算手掌朝向方向（从手腕到中指MCP）
            palm_direction = middle_mcp - wrist
            palm_direction_norm = np.linalg.norm(palm_direction)
            
            if palm_direction_norm < 1e-6:
                rospy.logwarn("Invalid palm direction, using default orientation")
                return Quaternion(0, 0, 0, 1)
            
            palm_direction = palm_direction / palm_direction_norm
            
            # 方法3：使用拇指方向作为参考（提高稳定性）
            thumb_direction = thumb_cmc - wrist
            thumb_direction_norm = np.linalg.norm(thumb_direction)
            
            if thumb_direction_norm > 1e-6:
                thumb_direction = thumb_direction / thumb_direction_norm
                # 使用拇指方向来调整坐标系
                side_direction = np.cross(palm_normal, palm_direction)
                side_direction = side_direction / np.linalg.norm(side_direction)
                
                # 构建更稳定的坐标系
                z_axis = -palm_normal  # Z轴指向手掌法线方向（手掌朝前）
                y_axis = palm_direction  # Y轴指向手指方向
                x_axis = np.cross(y_axis, z_axis)  # X轴为侧向
                x_axis = x_axis / np.linalg.norm(x_axis)
                
                # 重新正交化
                y_axis = np.cross(z_axis, x_axis)
                y_axis = y_axis / np.linalg.norm(y_axis)
            else:
                # 备用方法：使用简单的坐标系
                z_axis = -palm_normal
                x_axis = np.cross([0, 1, 0], z_axis)  # 使用世界Y轴作为参考
                if np.linalg.norm(x_axis) < 1e-6:
                    x_axis = np.cross([1, 0, 0], z_axis)
                x_axis = x_axis / np.linalg.norm(x_axis)
                y_axis = np.cross(z_axis, x_axis)
            
            # 构建旋转矩阵并检查正交性
            rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
            
            # 确保旋转矩阵是正交的
            if not self.is_valid_rotation_matrix(rotation_matrix):
                rospy.logwarn("Invalid rotation matrix, using previous orientation")
                return self.last_valid_orientation if hasattr(self, 'last_valid_orientation') else Quaternion(0, 0, 0, 1)
            
            # 转换为四元数
            quaternion = self.rotation_matrix_to_quaternion_stable(rotation_matrix)
            
            # 存储最后一次有效的方向
            self.last_valid_orientation = quaternion
            
            return quaternion
            
        except Exception as e:
            rospy.logwarn(f"Error calculating hand orientation: {str(e)}")
            return self.last_valid_orientation if hasattr(self, 'last_valid_orientation') else Quaternion(0, 0, 0, 1)

    def is_valid_rotation_matrix(self, R):
        """检查旋转矩阵是否有效"""
        # 检查行列式是否接近1
        det = np.linalg.det(R)
        if abs(det - 1.0) > 0.01:
            return False
        
        # 检查是否正交
        I = np.eye(3)
        product = np.dot(R, R.T)
        if not np.allclose(product, I, atol=0.1):
            return False
        
        return True

    def rotation_matrix_to_quaternion_stable(self, R):
        """稳定的旋转矩阵到四元数转换"""
        # 确保矩阵是正交的
        U, S, Vt = np.linalg.svd(R)
        R = np.dot(U, Vt)
        
        # 使用更稳定的转换方法
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = math.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
        
        # 创建四元数并确保单位化
        quat = np.array([x, y, z, w])
        quat = quat / np.linalg.norm(quat)
        
        # 确保w分量为正（标准形式）
        if quat[3] < 0:
            quat = -quat
        
        return Quaternion(quat[0], quat[1], quat[2], quat[3])

    def recognize_gesture(self, hand_landmarks):
        # 获取关键点
        landmarks = hand_landmarks.landmark
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        
        # 计算手掌基准大小（手腕到中指根部的距离）
        middle_mcp = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        palm_size = self.calculate_distance(wrist, middle_mcp)
        
        # 获取各指尖
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = landmarks[self.mp_hands.HandLandmark.PINKY_TIP]
        
        # 计算指尖与拇指尖的相对距离（相对于手掌大小）
        thumb_index_dist = self.calculate_distance(thumb_tip, index_tip) / palm_size
        thumb_index_dist_new = self.calculate_distance_new(thumb_tip, index_tip) / palm_size
        thumb_middle_dist = self.calculate_distance(thumb_tip, middle_tip) / palm_size
        thumb_ring_dist = self.calculate_distance(thumb_tip, ring_tip) / palm_size
        thumb_pinky_dist = self.calculate_distance(thumb_tip, pinky_tip) / palm_size
        
        # 计算各指尖是否弯曲（与对应MCP关节的距离）
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]
        middle_mcp = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        ring_mcp = landmarks[self.mp_hands.HandLandmark.RING_FINGER_MCP]
        pinky_mcp = landmarks[self.mp_hands.HandLandmark.PINKY_MCP]
        
        index_extended = self.calculate_distance(index_tip, wrist) > self.calculate_distance(index_mcp, wrist)
        middle_extended = self.calculate_distance(middle_tip, wrist) > self.calculate_distance(middle_mcp, wrist)
        ring_extended = self.calculate_distance(ring_tip, wrist) > self.calculate_distance(ring_mcp, wrist)
        pinky_extended = self.calculate_distance(pinky_tip, wrist) > self.calculate_distance(pinky_mcp, wrist)
        thumb_extended = self.calculate_distance(thumb_tip, wrist) > self.calculate_distance(
            landmarks[self.mp_hands.HandLandmark.THUMB_IP], wrist)
        
        self.gripper_cmd = (self.clamp(abs(thumb_index_dist_new), 0.7, 1.4)-0.7)/10
        if(self.gripper_cmd>=0.035):
            self.gripper_cmd = 0.07
        else:
            self.gripper_cmd = 0
        print(self.gripper_cmd)
        
        # 判断手势
        if all([index_extended, middle_extended, ring_extended, pinky_extended, thumb_extended]):
            return "OPEN_HAND"
        elif index_extended and middle_extended and not ring_extended and not pinky_extended:
            return "VICTORY"
        elif not any([index_extended, middle_extended, ring_extended, pinky_extended]):
            return "FIST"
        elif thumb_index_dist < 0.3 and all([not middle_extended, not ring_extended, not pinky_extended]):
            return "OK"
        else:
            return "UNKNOWN"

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def create_marker_array(self, landmarks_3d, bbox_center):
        """创建包含所有关键点的MarkerArray"""
        marker_array = MarkerArray()
        
        # 删除之前的标记（可选）
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 为每个关键点创建Marker
        for idx, (x, y, z) in enumerate(landmarks_3d):
            marker = Marker()
            marker.header.frame_id = "camera_color_optical_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "hand_landmarks"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02  # 2cm直径
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # 红色
            marker.lifetime = rospy.Duration(0.1)
            marker_array.markers.append(marker)
        
        # 添加边界框中心点标记
        if bbox_center:
            marker = Marker()
            marker.header.frame_id = "camera_color_optical_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "hand_bbox_center"
            marker.id = len(landmarks_3d) + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = bbox_center[0]
            marker.pose.position.y = bbox_center[1]
            marker.pose.position.z = bbox_center[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04  # 4cm直径
            marker.scale.y = 0.04
            marker.scale.z = 0.04
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # 绿色
            marker.lifetime = rospy.Duration(0.1)
            marker_array.markers.append(marker)
            
        return marker_array

    def publish_base_frame_origin(self, origin, rotation):
        """发布基坐标系原点"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "camera_color_optical_frame"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = origin[0]
        pose_msg.pose.position.y = origin[1]
        pose_msg.pose.position.z = origin[2]
        # pose_msg.pose.orientation = rotation
        pose_msg.pose.orientation.w = 1
        self.base_frame_pub.publish(pose_msg)
        
        transform1 = TransformStamped()
        transform1.header.stamp = rospy.Time.now()
        transform1.header.frame_id = "camera_color_optical_frame"
        transform1.child_frame_id = "base_frame_origin"
        transform1.transform.translation.x = self.base_frame_origin[0]
        transform1.transform.translation.y = self.base_frame_origin[1]
        transform1.transform.translation.z = self.base_frame_origin[2]
        # transform1.transform.rotation = rotation
        transform1.transform.rotation.w = 1
        self.tf_broadcaster.sendTransform(transform1)

    def color_callback(self, msg):
        # 检查是否已收到相机内参
        if not self.camera_info_received:
            rospy.logwarn_throttle(5.0, "Waiting for camera parameters...")
            return
            
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting color image: %s", str(e))
            return
        
        if self.latest_depth is None:
            return
            
        self.count += 1
        # 处理手势识别
        image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        marked_image = color_image.copy()
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # 绘制2D关键点
                self.mp_drawing.draw_landmarks(
                    marked_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                desired_point_indices = [0, 5, 9, 13, 17]  # 示例：手掌根部、指尖等关键点
                
                # 计算所有关键点的3D坐标
                landmarks_3d = []
                for landmark in hand_landmarks.landmark:
                    u = int(landmark.x * color_image.shape[1])
                    v = int(landmark.y * color_image.shape[0])
                    
                    # 获取深度值（注意检查边界）
                    if (self.latest_depth is not None and 
                        0 <= v < self.latest_depth.shape[0] and 
                        0 <= u < self.latest_depth.shape[1]):
                        depth = self.latest_depth[v, u]
                        point_3d = self.pixel_to_3d(u, v, depth)
                        if point_3d:
                            landmarks_3d.append(point_3d)
                
                selected_landmarks_3d = []
                for idx, landmark in enumerate(hand_landmarks.landmark):
                    # 只处理你想要的点
                    if idx in desired_point_indices:
                        u = int(landmark.x * color_image.shape[1])
                        v = int(landmark.y * color_image.shape[0])
                                        # 获取深度值（注意检查边界）
                        if (self.latest_depth is not None and 
                            0 <= v < self.latest_depth.shape[0] and 
                            0 <= u < self.latest_depth.shape[1]):
                            depth = self.latest_depth[v, u]
                            point_3d = self.pixel_to_3d(u, v, depth)
                            if point_3d:
                                selected_landmarks_3d.append(point_3d)
                
                # 使用numpy计算平均值
                if selected_landmarks_3d:
                    points_array = np.array(selected_landmarks_3d)
                    center_point = np.mean(points_array, axis=0)
                    print(f"中心点坐标: {center_point}")
                    self.centroid = (center_point[0], center_point[1], center_point[2])
                                        
                # 计算手掌边界框中心点
                # self.centroid = self.calculate_hand_bbox_center(hand_landmarks, color_image.shape)
                # self.centroid = (center_point[0], center_point[1], center_point[2])
                
                # 计算手的旋转方向
                hand_rotation = None
                if landmarks_3d and len(landmarks_3d) > self.mp_hands.HandLandmark.PINKY_MCP:
                    hand_rotation = self.calculate_hand_orientation(hand_landmarks, landmarks_3d)

                # 应用平滑滤波
                if self.centroid and hand_rotation:
                    smoothed_centroid = self.smooth_position(self.centroid)
                    smoothed_rotation = self.smooth_rotation(hand_rotation)
                    
                    rospy.loginfo(f"Raw hand bbox center: {self.centroid}")
                    rospy.loginfo(f"Smoothed hand bbox center: {smoothed_centroid}")
                    rospy.loginfo(f"Smoothed hand rotation: {smoothed_rotation}")
                    
                    # 使用平滑后的数据
                    self.centroid = smoothed_centroid
                    hand_rotation = smoothed_rotation
                
                if self.centroid and hand_rotation:
                    rospy.loginfo(f"Hand bbox center: {self.centroid}")
                    rospy.loginfo(f"Hand rotation: {hand_rotation}")
                    
                    # 如果有基坐标系，计算形心相对于基坐标系的坐标
                    if self.base_frame_origin is not None:
                        relative_x = self.centroid[0] - self.base_frame_origin[0]
                        relative_y = self.centroid[1] - self.base_frame_origin[1]
                        relative_z = self.centroid[2] - self.base_frame_origin[2]
                        rospy.loginfo(f"Relative to base frame: X={relative_x:.3f}, Y={relative_y:.3f}, Z={relative_z:.3f}")

                        hand_target_pose = PoseStamped()
                        hand_target_pose.pose.position.x = self.centroid[0]
                        hand_target_pose.pose.position.y = self.centroid[1]
                        hand_target_pose.pose.position.z = self.centroid[2]
                        hand_target_pose.pose.orientation = hand_rotation

                        # if self.count % 3 == 0:
                        if self.last_gesture == "OPEN_HAND":
                            if abs(relative_x)>=100 or abs(relative_y)>=100 or abs(relative_z)>=100:
                                rospy.logwarn("abs>100")
                            self.publish_target_pose(relative_x*1.5, relative_y*1.5, relative_z*1.5, hand_rotation)
                            transform2 = TransformStamped()
                            transform2.header.stamp = rospy.Time.now()
                            transform2.header.frame_id = "camera_color_optical_frame"
                            transform2.child_frame_id = "hand_target_pose"
                            transform2.transform.translation.x = hand_target_pose.pose.position.x
                            transform2.transform.translation.y = hand_target_pose.pose.position.y
                            transform2.transform.translation.z = hand_target_pose.pose.position.z
                            transform2.transform.rotation = hand_target_pose.pose.orientation
                            self.tf_broadcaster.sendTransform(transform2)
                
                # 识别手势
                current_gesture = self.recognize_gesture(hand_landmarks)
                
                # 跟踪手势持续时间
                if current_gesture == self.last_gesture:
                    print(current_gesture)
                    if current_gesture == "FIST":
                        current_time = time.time()
                        if self.gesture_start_time is None:
                            self.gesture_start_time = current_time
                        elif current_time - self.gesture_start_time >= 3.0:  # 保持3秒
                            if self.centroid and hand_rotation:  # 设置基坐标系原点
                                self.base_frame_origin = self.centroid
                                self.base_frame_rotation = hand_rotation
                                self.publish_base_frame_origin(self.centroid, hand_rotation)
                                rospy.loginfo(f"Base frame origin set to: {self.centroid}")
                                self.gesture_start_time = None  # 重置计时器
                    # elif current_gesture == "VICTORY":
                    #     current_time = time.time()
                    #     if self.gesture_start_time is None:
                    #         self.gesture_start_time = current_time
                    #     elif current_time - self.gesture_start_time >= 0.2:  # 保持0.5秒
                    #         if self.gripper_cmd>0:
                    #             self.gripper_cmd=0
                    #         else:
                    #             self.gripper_cmd=0.07
                    #         self.gesture_start_time = None  # 重置计时器
                else:
                    self.last_gesture = current_gesture
                    self.gesture_start_time = None
                
                # 发布MarkerArray
                if landmarks_3d:
                    marker_array = self.create_marker_array(landmarks_3d, self.centroid)
                    self.marker_pub.publish(marker_array)
        
        # 发布标记后的图像
        try:
            marked_img_msg = self.bridge.cv2_to_imgmsg(marked_image, "bgr8")
            self.marked_image_pub.publish(marked_img_msg)
        except Exception as e:
            rospy.logerr("Error publishing marked image: %s", str(e))

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            rospy.logerr("Error converting depth image: %s", str(e))

    def endPostion_callback(self, msg):
        if msg and self.is_record_arm_endpos:
            self.is_end_pose_recieved = True
            self.armEndPos = msg
            transform3 = TransformStamped()
            transform3.header.stamp = rospy.Time.now()
            transform3.header.frame_id = "base_link"
            transform3.child_frame_id = "end_pose"
            transform3.transform.translation.x = self.armEndPos.pose.position.x
            transform3.transform.translation.y = self.armEndPos.pose.position.y
            transform3.transform.translation.z = self.armEndPos.pose.position.z
            transform3.transform.rotation = self.armEndPos.pose.orientation
            self.tf_broadcaster.sendTransform(transform3)

    def publish_target_pose(self, relativeX, relativeY, relativeZ, hand_rotation):
        if (not self.is_end_pose_recieved):
            transform3 = TransformStamped()
            transform3.header.stamp = rospy.Time.now()
            transform3.header.frame_id = "base_link"
            transform3.child_frame_id = "end_pose"
            transform3.transform.translation.x = self.armEndPos.pose.position.x
            transform3.transform.translation.y = self.armEndPos.pose.position.y
            transform3.transform.translation.z = self.armEndPos.pose.position.z
            transform3.transform.rotation = self.armEndPos.pose.orientation
            self.tf_broadcaster.sendTransform(transform3)
        
        if (abs(relativeX) > 1.5 or abs(relativeY) > 1.5 or abs(relativeZ) > 1.5):
            rospy.logwarn(f"Large movement detected, ignoring: {relativeX}, {relativeY}, {relativeZ}")
            return

        targetX = self.armEndPos.pose.position.x + relativeX
        targetY = self.armEndPos.pose.position.y + relativeY
        targetZ = self.armEndPos.pose.position.z + relativeZ
        
        targetPose = PoseStamped()

        targetPose.header.stamp = rospy.Time.now()
        targetPose.header.frame_id = "base_link"
        targetPose.pose.position.x = targetX
        targetPose.pose.position.y = targetY
        targetPose.pose.position.z = targetZ
        
        # 旋转映射：计算手部相对于基坐标系的相对旋转，然后应用到机械臂末端
        if self.base_frame_rotation is not None:
            # 将四元数转换为numpy数组
            q_base = np.array([self.base_frame_rotation.x, self.base_frame_rotation.y, 
                            self.base_frame_rotation.z, self.base_frame_rotation.w])
            q_hand = np.array([hand_rotation.x, hand_rotation.y, 
                            hand_rotation.z, hand_rotation.w])
            q_arm_end = np.array([self.armEndPos.pose.orientation.x, self.armEndPos.pose.orientation.y,
                                self.armEndPos.pose.orientation.z, self.armEndPos.pose.orientation.w])
            
            # 计算手部相对于基坐标系的旋转
            # relative_rotation = q_hand * conjugate(q_base)
            q_base_conj = tf_trans.quaternion_conjugate(q_base)
            relative_rotation = tf_trans.quaternion_multiply(q_hand, q_base_conj)
            
            # 将相对旋转应用到机械臂末端的当前旋转
            # final_rotation = relative_rotation * q_arm_end
            final_quaternion = tf_trans.quaternion_multiply(relative_rotation, q_arm_end)
            
            # 归一化四元数
            final_quaternion = tf_trans.unit_vector(final_quaternion)
            
            # 设置最终的四元数
            targetPose.pose.orientation.x = final_quaternion[0]
            targetPose.pose.orientation.y = final_quaternion[1]
            targetPose.pose.orientation.z = final_quaternion[2]
            targetPose.pose.orientation.w = final_quaternion[3]

        else:
            # 如果没有基坐标系旋转，使用手部旋转（保持原有逻辑）
            q_hand = [hand_rotation.x, hand_rotation.y, hand_rotation.z, hand_rotation.w]
            z_rotation_90 = tf_trans.quaternion_about_axis(math.pi/2, [0, 0, 1])
            final_quaternion = tf_trans.quaternion_multiply(q_hand, z_rotation_90)
            targetPose.pose.orientation.x = final_quaternion[0]
            targetPose.pose.orientation.y = final_quaternion[1]
            targetPose.pose.orientation.z = final_quaternion[2]
            targetPose.pose.orientation.w = final_quaternion[3]
        
        self.pos_cmd_pub.publish(targetPose)

        transform4 = TransformStamped()
        transform4.header.stamp = rospy.Time.now()
        transform4.header.frame_id = "base_link"
        transform4.child_frame_id = "target_end_pose"
        transform4.transform.translation.x = targetPose.pose.position.x
        transform4.transform.translation.y = targetPose.pose.position.y
        transform4.transform.translation.z = targetPose.pose.position.z
        transform4.transform.rotation = targetPose.pose.orientation
        self.tf_broadcaster.sendTransform(transform4)
        self.publish_gripper_control()

    # def shutdown(self):
    #     rospy.loginfo("Shutting down gesture recognizer")
    #     self.hands.close()

    def run(self):
        tty.setcbreak(sys.stdin.fileno())
        
        while not rospy.is_shutdown():
            key = self.get_key()
            if key:
                if key == 'r':
                    if self.is_record_arm_endpos == True:
                        self.is_record_arm_endpos = False
                    elif self.is_record_arm_endpos == False:
                        self.is_record_arm_endpos = True
                    print("Record arm end position...")
                
                elif key == 'p':
                    pass
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        recognizer = GestureRecognizer()
        recognizer.run()
    except rospy.ROSInterruptException:
        pass