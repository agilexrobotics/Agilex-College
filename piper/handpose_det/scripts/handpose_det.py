#!/usr/bin/env python3
# _*_ coding:utf-8 _*_

import rospy
import cv2
import mediapipe as mp
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
import time
from piper_msgs.msg import PiperStatusMsg, PosCmd, PiperEulerPose


class GestureRecognizer:
    def __init__(self):
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
        
        # 相机参数（已对齐深度与RGB）奥比中光
        self.fx = 512.8  # 水平焦距 (像素)//470.3 
        self.fy = 483.1  # 垂直焦距 (像素)
        # self.cx = 320    # 主点x (像素)
        # self.cy = 200    # 主点y (像素)

        self.cx = 320    # 主点x (像素)
        self.cy = 240    # 主点y (像素)

        # 记录机械臂末端的坐标
        self.endPosX = -0.344
        self.endPosY = 0.0
        self.endPosZ = 0.110

        # 手掌边界框中心坐标
        self.centroid = (0, 0, 0)
        
        # 订阅图像话题
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.endPosition_sub = rospy.Subscriber("/end_pose", PoseStamped, self.endPostion_callback)
        
        # 发布带标记的图像
        self.marked_image_pub = rospy.Publisher("/gesture_recognition/marked_image", Image, queue_size=1)
        # 发布手部关键点MarkerArray
        self.marker_pub = rospy.Publisher("/hand_landmarks_markers", MarkerArray, queue_size=1)
        # 发布基坐标系原点
        self.base_frame_pub = rospy.Publisher("/base_frame_origin", PoseStamped, queue_size=1)
        # 发布相对基坐标系原点的相对位置
        self.pos_cmd_pub = rospy.Publisher("/pos_cmd", PosCmd, queue_size=10)


        # 存储最新的深度图
        self.latest_depth = None
        
        # 手势状态跟踪
        self.last_gesture = None
        self.gesture_start_time = None
        self.base_frame_origin = None  # 存储基坐标系原点 [x, y, z]
        
        rospy.loginfo("Gesture Recognizer initialized")

    def pixel_to_3d(self, u, v, depth):
        """将像素坐标+深度值转换为3D坐标(相机坐标系)"""
        if depth == 0:
            return None
        
        Z = depth / 1000.0  # 转换为米
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return (X, Y, Z)
    
    def calculate_distance(self, p1, p2):
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)

    def calculate_hand_bbox_center(self, hand_landmarks, image_shape):
        """计算手掌边界框的中心点"""
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
        if 0 <= center_y < self.latest_depth.shape[0] and 0 <= center_x < self.latest_depth.shape[1]:
            depth = self.latest_depth[center_y, center_x]
            point_3d = self.pixel_to_3d(center_x, center_y, depth)
            return point_3d
        return None

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

    def publish_base_frame_origin(self, origin):
        """发布基坐标系原点"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "camera_color_optical_frame"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = origin[0]
        pose_msg.pose.position.y = origin[1]
        pose_msg.pose.position.z = origin[2]
        pose_msg.pose.orientation.w = 1.0  # 无旋转
        self.base_frame_pub.publish(pose_msg)

    def color_callback(self, msg):
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
                
                # 计算所有关键点的3D坐标
                landmarks_3d = []
                for landmark in hand_landmarks.landmark:
                    u = int(landmark.x * color_image.shape[1])
                    v = int(landmark.y * color_image.shape[0])
                    
                    # 获取深度值（注意检查边界）
                    if 0 <= v < self.latest_depth.shape[0] and 0 <= u < self.latest_depth.shape[1]:
                        depth = self.latest_depth[v, u]
                        point_3d = self.pixel_to_3d(u, v, depth)
                        if point_3d:
                            landmarks_3d.append(point_3d)
                
                # 计算手掌边界框中心点
                self.centroid = self.calculate_hand_bbox_center(hand_landmarks, color_image.shape)
                if self.centroid:
                    rospy.loginfo(f"Hand bbox center: {self.centroid}")
                    
                    # 如果有基坐标系，计算形心相对于基坐标系的坐标
                    if self.base_frame_origin is not None:
                        relative_x = self.centroid[0] - self.base_frame_origin[0]
                        relative_y = self.centroid[1] - self.base_frame_origin[1]
                        relative_z = self.centroid[2] - self.base_frame_origin[2]
                        rospy.loginfo(f"Relative to base frame: X={relative_x:.3f}, Y={relative_y:.3f}, Z={relative_z:.3f}")

                        if self.count % 3 == 0:
                            if abs(relative_x)>=100 or abs(relative_y)>=100 or abs(relative_z)>=100:
                                rospy.logwarn("abs>100")
                            self.publish_target_pose(relative_x, relative_y, relative_z)
                
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
                            if self.centroid:  # 设置基坐标系原点
                                self.base_frame_origin = self.centroid
                                self.publish_base_frame_origin(self.centroid)
                                rospy.loginfo(f"Base frame origin set to: {self.centroid}")
                                self.gesture_start_time = None  # 重置计时器
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
        # self.endPosX = msg.pose.position.x
        # self.endPosY = msg.pose.position.y
        # self.endPosZ = msg.pose.position.z
        self.endPosX = -0.344
        self.endPosY = 0.0
        self.endPosZ = 0.110
        return
    
    def publish_target_pose(self, relativeX, relativeY, relativeZ):
        targetX = self.endPosX + relativeY
        targetY = self.endPosY + relativeX
        targetZ = self.endPosZ - relativeZ
        if abs(targetX)>=100 or abs(targetY)>=100 or abs(targetZ)>=100:
            rospy.logwarn("abs>100")
        targetPose = PosCmd()
        targetPose.x = targetX
        targetPose.y = targetY
        targetPose.z = targetZ
        targetPose.pitch = 0
        targetPose.yaw = 0
        targetPose.roll = 0
        targetPose.gripper = 1
        targetPose.mode1 = 1
        targetPose.mode2 = 0
        self.pos_cmd_pub.publish(targetPose)

    def shutdown(self):
        rospy.loginfo("Shutting down gesture recognizer")
        self.hands.close()

if __name__ == '__main__':
    rospy.init_node('gesture_recognizer')
    recognizer = GestureRecognizer()
    rospy.on_shutdown(recognizer.shutdown)
    rospy.spin()
