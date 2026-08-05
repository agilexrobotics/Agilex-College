#!/usr/bin/env python3
"""Dependency-light face detector used when the teammate YOLO model is absent."""

import os
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


class FaceCenterDetector(Node):
    def __init__(self) -> None:
        super().__init__("face_center_detector")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("landmarks_topic", "/face_landmarks_2d")
        self.declare_parameter("debug_topic", "/face_landmarks_debug_image")
        self.declare_parameter(
            "cascade_path",
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt2.xml",
        )
        self.declare_parameter("max_fps", 12.0)
        self.declare_parameter("min_face_px", 70)
        cascade_path = str(self.get_parameter("cascade_path").value)
        if not os.path.isfile(cascade_path):
            raise FileNotFoundError(f"OpenCV face cascade not found: {cascade_path}")
        self._cascade = cv2.CascadeClassifier(cascade_path)
        if self._cascade.empty():
            raise RuntimeError(f"cannot load face cascade: {cascade_path}")
        self._bridge = CvBridge()
        self._period = 1.0 / max(1.0, float(self.get_parameter("max_fps").value))
        self._last_run = 0.0
        self._min_face = int(self.get_parameter("min_face_px").value)
        self._landmarks_pub = self.create_publisher(
            Float32MultiArray, str(self.get_parameter("landmarks_topic").value), 10
        )
        self._debug_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), 10
        )
        self._status_pub = self.create_publisher(String, "/piper_x_face_follow/perception_status", 10)
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info("Face detector ready (OpenCV fallback, no external model needed).")

    def _image_callback(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_run < self._period:
            return
        self._last_run = now
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        found = self._cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(self._min_face, self._min_face),
        )
        faces = sorted(found, key=lambda item: int(item[2]) * int(item[3]), reverse=True)
        status = String()
        status.data = f"faces={len(faces)}"
        self._status_pub.publish(status)
        if faces:
            x, y, width, height = [int(value) for value in faces[0]]
            center_u = x + width * 0.5
            center_v = y + height * 0.5
            # Match the teammate package's 13-value layout. Approximate eye/nose/mouth
            # coordinates are sufficient because the controller only consumes [9:11].
            output = Float32MultiArray()
            output.data = [
                x + width * 0.32, y + height * 0.40,
                x + width * 0.68, y + height * 0.40,
                center_u, y + height * 0.56,
                center_u, y + height * 0.76,
                1.0, center_u, center_v, float(width), float(height),
            ]
            self._landmarks_pub.publish(output)
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
            cv2.circle(frame, (round(center_u), round(center_v)), 5, (0, 0, 255), -1)
        debug = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug.header = msg.header
        self._debug_pub.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceCenterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
