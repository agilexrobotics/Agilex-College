#!/usr/bin/env python3
"""MediaPipe body/hands fused with registered Orbbec depth.

MediaPipe supplies reliable pixel landmarks and association. Absolute geometry
always comes from the depth camera and CameraInfo, never MediaPipe's relative
"world" coordinates.
"""

from collections import deque
import json
import math
import threading
import time
from types import SimpleNamespace
from typing import Dict, Optional, Sequence, Tuple

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from smart_keyboard_msgs.msg import UserState
from std_msgs.msg import Bool, Float32, String, UInt8
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


POSE = mp.solutions.holistic.PoseLandmark


def median_depth_m(
    depth: np.ndarray,
    encoding: str,
    u: int,
    v: int,
    radius: int,
    minimum: float,
    maximum: float,
) -> Optional[float]:
    if depth is None or not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
        return None
    patch = depth[
        max(0, v - radius): min(depth.shape[0], v + radius + 1),
        max(0, u - radius): min(depth.shape[1], u + radius + 1),
    ].astype(np.float32)
    if encoding in ("16UC1", "mono16"):
        patch *= 0.001
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid >= minimum) & (valid <= maximum)]
    return float(np.median(valid)) if valid.size else None


def robust_landmark_depth_m(
    depth: np.ndarray,
    encoding: str,
    pixels: Sequence[Tuple[float, float]],
    radius: int,
    minimum: float,
    maximum: float,
    minimum_samples: int = 2,
) -> Optional[float]:
    """Estimate hand range from multiple palm points with MAD outlier gating."""
    values = [
        median_depth_m(
            depth,
            encoding,
            round(pixel[0]),
            round(pixel[1]),
            radius,
            minimum,
            maximum,
        )
        for pixel in pixels
    ]
    valid = np.asarray([value for value in values if value is not None], dtype=float)
    if valid.size < max(1, minimum_samples):
        return None
    center = float(np.median(valid))
    deviation = np.abs(valid - center)
    mad = float(np.median(deviation))
    # 35 mm minimum gate tolerates normal palm curvature while rejecting the
    # much larger hand/background discontinuity.
    gate = max(0.035, 3.5 * 1.4826 * mad)
    clustered = valid[deviation <= gate]
    if clustered.size < max(1, minimum_samples):
        return None
    return float(np.median(clustered))


def deproject(info: CameraInfo, u: float, v: float, depth_m: float) -> np.ndarray:
    fx, fy = float(info.k[0]), float(info.k[4])
    cx, cy = float(info.k[2]), float(info.k[5])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("invalid CameraInfo focal length")
    return np.array(
        [(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m],
        dtype=np.float64,
    )


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3)
    scale = 2.0 / norm
    return np.array(
        [
            [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_quaternion(matrix: np.ndarray) -> Tuple[float, float, float, float]:
    """Return an xyzw quaternion for a proper 3x3 rotation matrix."""
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("rotation matrix must be 3x3")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
            ) * 2.0
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
            w = (rotation[2, 1] - rotation[1, 2]) / scale
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
            ) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
            w = (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
            ) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
            w = (rotation[1, 0] - rotation[0, 1]) / scale
    quaternion = np.asarray([x, y, z, w], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def typing_plane_rotation(
    left_center: Sequence[float],
    right_center: Sequence[float],
    left_forward: Sequence[float],
    right_forward: Sequence[float],
) -> np.ndarray:
    """Build the two-hand typing frame: X lateral, Y fingers, Z plane normal."""
    left = np.asarray(left_center, dtype=float)
    right = np.asarray(right_center, dtype=float)
    midpoint = 0.5 * (left + right)
    lateral = right - left
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm < 0.06:
        raise ValueError("hands are too close for a typing plane")
    lateral /= lateral_norm
    forward = np.asarray(left_forward, dtype=float) + np.asarray(
        right_forward, dtype=float
    )
    forward -= lateral * float(np.dot(forward, lateral))
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm < 0.025:
        raise ValueError("hand forward axes are degenerate")
    forward /= forward_norm
    normal = np.cross(lateral, forward)
    normal /= np.linalg.norm(normal)
    # The useful side of a typing plane faces the camera.
    if float(np.dot(normal, -midpoint)) < 0.0:
        forward = -forward
        normal = -normal
    return np.column_stack((lateral, forward, normal))


def hand_layout_features(left_landmarks, right_landmarks) -> dict:
    """Return image-space geometry used only for conservative camera yaw/roll.

    The entries are ordered by their position in the image, not MediaPipe's
    anatomical handedness.  This remains well-defined for a mirrored image.
    """
    hands = [
        hand for hand in (left_landmarks, right_landmarks) if hand is not None
    ]
    records = []
    for hand in hands:
        xy = np.asarray(
            [[landmark.x, landmark.y] for landmark in hand.landmark], dtype=float
        )
        center = np.mean(xy[[0, 5, 9, 13, 17]], axis=0)
        forward = xy[12] - xy[0]
        forward_angle = math.atan2(float(forward[1]), float(forward[0]))
        inside = np.logical_and(xy >= 0.035, xy <= 0.965)
        in_frame_fraction = float(np.mean(np.all(inside, axis=1)))
        records.append(
            {
                "center": center,
                "all_21_in_frame": bool(np.all(inside)),
                "in_frame_fraction": in_frame_fraction,
                "forward_angle_rad": float(forward_angle),
            }
        )
    records.sort(key=lambda item: float(item["center"][0]))
    payload = {
        "visible_count": len(records),
        "screen_left": None,
        "screen_right": None,
        "both_complete": False,
        "usable_pair": False,
        "midpoint_x": None,
        "midpoint_y": None,
        "separation_x": None,
        "hand_line_angle_rad": None,
        "finger_parallel_error_rad": None,
    }
    for key, record in zip(("screen_left", "screen_right"), records):
        payload[key] = {
            "x": float(record["center"][0]),
            "y": float(record["center"][1]),
            "all_21_in_frame": record["all_21_in_frame"],
            "in_frame_fraction": record["in_frame_fraction"],
            "forward_angle_rad": record["forward_angle_rad"],
        }
    if len(records) == 2:
        left, right = records
        delta = right["center"] - left["center"]
        payload.update(
            {
                "both_complete": bool(
                    left["all_21_in_frame"] and right["all_21_in_frame"]
                ),
                "usable_pair": bool(
                    left["in_frame_fraction"] >= 0.75
                    and right["in_frame_fraction"] >= 0.75
                    and float(delta[0]) >= 0.10
                ),
                "midpoint_x": float(
                    0.5 * (left["center"][0] + right["center"][0])
                ),
                "midpoint_y": float(
                    0.5 * (left["center"][1] + right["center"][1])
                ),
                "separation_x": float(delta[0]),
                "hand_line_angle_rad": float(
                    math.atan2(float(delta[1]), max(float(delta[0]), 1.0e-6))
                ),
                "finger_parallel_error_rad": float(
                    math.atan2(
                        math.sin(
                            right["forward_angle_rad"] - left["forward_angle_rad"]
                        ),
                        math.cos(
                            right["forward_angle_rad"] - left["forward_angle_rad"]
                        ),
                    )
                ),
            }
        )
    return payload


def one_two_gesture(hand_landmarks) -> Optional[str]:
    """Recognize an intentional one- or two-finger command gesture.

    Thumb state is ignored.  A finger must be both straight and extend farther
    from the wrist than its PIP joint.  Ring and little fingers must be folded,
    which avoids treating an ordinary open typing hand as a command.
    """
    if hand_landmarks is None or len(hand_landmarks.landmark) < 21:
        return None
    xy = np.asarray(
        [[item.x, item.y] for item in hand_landmarks.landmark], dtype=float
    )
    if not np.all(np.isfinite(xy)):
        return None
    wrist = xy[0]
    palm_scale = float(np.linalg.norm(xy[9] - wrist))
    if palm_scale < 1.0e-4:
        return None

    def extended(
        mcp: int, pip: int, tip: int, *, strong: bool = False
    ) -> bool:
        tip_wrist = float(np.linalg.norm(xy[tip] - wrist))
        pip_wrist = float(np.linalg.norm(xy[pip] - wrist))
        tip_mcp = float(np.linalg.norm(xy[tip] - xy[mcp]))
        pip_mcp = float(np.linalg.norm(xy[pip] - xy[mcp]))
        # The commanded finger uses a relaxed threshold so foreshortening and
        # a slightly bent fingertip still work. Unused fingers only reject the
        # gesture when they are clearly/strongly extended, so they need not be
        # clenched completely.
        wrist_ratio = 1.10 if strong else 1.03
        mcp_ratio = 1.12 if strong else 1.04
        palm_margin = 0.08 if strong else 0.03
        return (
            tip_wrist >= wrist_ratio * max(pip_wrist, 1.0e-6)
            and tip_mcp >= mcp_ratio * max(pip_mcp, 1.0e-6)
            and tip_wrist - pip_wrist >= palm_margin * palm_scale
        )

    index = extended(5, 6, 8)
    middle = extended(9, 10, 12)
    middle_strong = extended(9, 10, 12, strong=True)
    ring_strong = extended(13, 14, 16, strong=True)
    little_strong = extended(17, 18, 20, strong=True)
    if index and middle and not ring_strong and not little_strong:
        return "two"
    if index and not middle_strong and not ring_strong and not little_strong:
        return "one"
    return None


def exclusive_command_gesture(gesture_results) -> Optional[str]:
    """Accept a command only when exactly one visible hand presents it."""
    valid = [value for value in gesture_results if value in ("one", "two")]
    return valid[0] if len(valid) == 1 else None


class MediaPipeBodyHandNode(Node):
    def __init__(self) -> None:
        super().__init__("mediapipe_body_hands")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("max_fps", 12.0)
        self.declare_parameter("depth_sync_tolerance_sec", 0.20)
        self.declare_parameter("max_latest_depth_age_sec", 0.20)
        self.declare_parameter("depth_window_radius", 4)
        self.declare_parameter("min_depth_m", 0.20)
        self.declare_parameter("max_depth_m", 3.00)
        self.declare_parameter("min_visibility", 0.55)
        self.declare_parameter("state_stable_samples", 8)
        self.declare_parameter("side_threshold_norm", 0.10)
        self.declare_parameter("standing_hip_knee_norm", 0.16)
        self.declare_parameter("hands_on_keyboard_max_depth_m", 0.75)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("tracking_center_vertical_offset_deg", -12.0)
        self.declare_parameter("hands_only_servo_mode", False)
        self.declare_parameter("hand_memory_sec", 0.35)
        self.declare_parameter("gesture_hold_sec", 3.0)
        self.declare_parameter("gesture_neutral_samples", 3)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._period = 1.0 / max(1.0, float(self.get_parameter("max_fps").value))
        self._sync_tolerance = float(self.get_parameter("depth_sync_tolerance_sec").value)
        self._max_latest_depth_age = float(
            self.get_parameter("max_latest_depth_age_sec").value
        )
        self._radius = int(self.get_parameter("depth_window_radius").value)
        self._min_depth = float(self.get_parameter("min_depth_m").value)
        self._max_depth = float(self.get_parameter("max_depth_m").value)
        self._min_visibility = float(self.get_parameter("min_visibility").value)
        self._side_threshold = float(self.get_parameter("side_threshold_norm").value)
        self._standing_threshold = float(
            self.get_parameter("standing_hip_knee_norm").value
        )
        self._hands_on_depth = float(
            self.get_parameter("hands_on_keyboard_max_depth_m").value
        )
        self._publish_debug = bool(self.get_parameter("publish_debug_image").value)
        self._tracking_vertical_offset = math.radians(
            float(
                self.get_parameter(
                    "tracking_center_vertical_offset_deg"
                ).value
            )
        )
        self._hands_only_servo = bool(
            self.get_parameter("hands_only_servo_mode").value
        )
        self._hand_memory_sec = max(
            0.0, float(self.get_parameter("hand_memory_sec").value)
        )
        self._last_hand_points: Dict[str, Tuple[np.ndarray, float]] = {}
        self._gesture_hold_sec = max(
            0.5, float(self.get_parameter("gesture_hold_sec").value)
        )
        self._gesture_neutral_required = max(
            2, int(self.get_parameter("gesture_neutral_samples").value)
        )
        self._gesture_neutral_count = self._gesture_neutral_required
        self._gesture_armed = True
        self._gesture_candidate = None
        self._timed_gesture_candidate = None
        self._gesture_candidate_since = 0.0
        self._last_logged_gesture_candidate = None
        self._last_gesture_command = ""
        self._state_votes = deque(
            maxlen=max(2, int(self.get_parameter("state_stable_samples").value))
        )
        self._stable_pose = "absent"
        self._state_since = time.monotonic()
        self._last_run = 0.0
        self._busy = False
        self._lock = threading.Lock()
        self._depth: Optional[np.ndarray] = None
        self._depth_encoding = ""
        self._depth_stamp_sec = 0.0
        self._depth_received_monotonic = 0.0
        self._camera_info: Optional[CameraInfo] = None
        self._last_status_signature = None
        self._bridge = CvBridge()
        self._callback_group = ReentrantCallbackGroup()

        self._holistic = (
            None
            if self._hands_only_servo
            else mp.solutions.holistic.Holistic(
                static_image_mode=False,
                model_complexity=1,
                smooth_landmarks=True,
                enable_segmentation=False,
                refine_face_landmarks=False,
                min_detection_confidence=0.55,
                min_tracking_confidence=0.55,
            )
        )
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=1,
            min_detection_confidence=0.50,
            min_tracking_confidence=0.50,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._user_pub = self.create_publisher(UserState, "/smart_keyboard/user_state", 10)
        self._status_pub = self.create_publisher(String, "/human_tracking/status", 10)
        self._debug_pub = self.create_publisher(Image, "/human_tracking/debug_image", 10)
        self._both_hands_pub = self.create_publisher(
            Bool, "/human_tracking/both_hands_3d", 10
        )
        self._hand_distance_pub = self.create_publisher(
            Float32, "/human_tracking/hand_distance_m", 10
        )
        self._typing_plane_pub = self.create_publisher(
            PoseStamped, "/human_tracking/typing_plane", 10
        )
        self._visual_target_pub = self.create_publisher(
            PointStamped, "/human_tracking/visual_hand_target", 10
        )
        self._visible_hand_count_pub = self.create_publisher(
            UInt8, "/human_tracking/visible_hand_count", 10
        )
        self._hand_layout_pub = self.create_publisher(
            String, "/human_tracking/hand_layout", 10
        )
        self._gesture_pub = self.create_publisher(
            String, "/human_tracking/gesture_command", 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/human_tracking/markers", 10
        )
        self._point_pubs = {
            name: self.create_publisher(PointStamped, f"/human_tracking/{name}", 10)
            for name in ("torso", "left_hand", "right_hand", "hand_midpoint")
        }
        self._base_point_pubs = {
            name: self.create_publisher(
                PointStamped, f"/human_tracking/{name}_base", 10
            )
            for name in ("torso", "left_hand", "right_hand", "hand_midpoint")
        }

        self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            self._color_cb,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_cb,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._info_cb,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        detector_mode = (
            "Hands-only low-latency servo"
            if self._hands_only_servo
            else "Holistic body+hands"
        )
        self.get_logger().info(
            f"MediaPipe {detector_mode} ready; absolute XYZ comes from Orbbec depth."
        )

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self._publish_status({"ok": False, "error": f"depth conversion: {exc}"})
            return
        with self._lock:
            self._depth = np.asarray(depth)
            self._depth_encoding = msg.encoding
            self._depth_stamp_sec = self._stamp_sec(msg.header.stamp)
            self._depth_received_monotonic = time.monotonic()

    def _info_cb(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _color_cb(self, msg: Image) -> None:
        now = time.monotonic()
        if self._busy or now - self._last_run < self._period:
            return
        self._busy = True
        try:
            self._process(msg)
            self._last_run = time.monotonic()
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().error(f"body/hand inference failed: {exc}")
                self._publish_status({"ok": False, "error": str(exc)})
        finally:
            self._busy = False

    def _process(self, msg: Image) -> None:
        info = self._camera_info
        with self._lock:
            depth = None if self._depth is None else self._depth.copy()
            depth_encoding = self._depth_encoding
            depth_stamp = self._depth_stamp_sec
            depth_age = time.monotonic() - self._depth_received_monotonic
        color_stamp = self._stamp_sec(msg.header.stamp)
        if info is None or depth is None:
            self._publish_absent(msg, "waiting_for_depth_or_camera_info")
            return
        stamp_delta = abs(color_stamp - depth_stamp)
        if (
            stamp_delta > self._sync_tolerance
            and depth_age > self._max_latest_depth_age
        ):
            self._publish_absent(msg, "color_depth_timestamp_mismatch")
            return

        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if depth.shape[:2] != bgr.shape[:2]:
            self._publish_absent(
                msg,
                f"depth_not_registered color={bgr.shape[:2]} depth={depth.shape[:2]}",
            )
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        result = (
            SimpleNamespace(
                pose_landmarks=None,
                left_hand_landmarks=None,
                right_hand_landmarks=None,
            )
            if self._hands_only_servo
            else self._holistic.process(rgb)
        )
        hands_result = self._hands.process(rgb)
        rgb.flags.writeable = True
        standalone_left, standalone_right = self._standalone_hands(hands_result)
        left_landmarks = standalone_left or result.left_hand_landmarks
        right_landmarks = standalone_right or result.right_hand_landmarks
        if (
            result.pose_landmarks is None
            and left_landmarks is None
            and right_landmarks is None
        ):
            self._publish_absent(msg, "no_person")
            self._draw_and_publish(msg, bgr, result, None, None)
            return

        landmark_hands = [
            hand
            for hand in (left_landmarks, right_landmarks)
            if hand is not None
        ]
        gesture_results = [
            value
            for value in (
                one_two_gesture(hand) for hand in landmark_hands
            )
            if value is not None
        ]
        # Both hands may remain visible, but exactly one of them must be
        # presenting a command. Two simultaneous command hands are ambiguous.
        gesture = exclusive_command_gesture(gesture_results)
        self._update_gesture(gesture)

        height, width = bgr.shape[:2]
        pose = (
            result.pose_landmarks.landmark
            if result.pose_landmarks is not None
            else None
        )
        pixels = self._pose_pixels(pose, width, height) if pose is not None else {}
        source_frame = info.header.frame_id or msg.header.frame_id
        points: Dict[str, np.ndarray] = {}

        shoulder_l = self._point3d(
            pixels.get("left_shoulder"), depth, depth_encoding, info
        )
        shoulder_r = self._point3d(
            pixels.get("right_shoulder"), depth, depth_encoding, info
        )
        hip_l = self._point3d(pixels.get("left_hip"), depth, depth_encoding, info)
        hip_r = self._point3d(pixels.get("right_hip"), depth, depth_encoding, info)
        torso_parts = [
            point
            for point in (shoulder_l, shoulder_r, hip_l, hip_r)
            if point is not None
        ]
        if len(torso_parts) >= 2:
            points["torso"] = np.mean(torso_parts, axis=0)

        raw_left_hand = self._hand_center(
            left_landmarks, pixels.get("left_wrist"),
            depth, depth_encoding, info, width, height,
        )
        raw_right_hand = self._hand_center(
            right_landmarks, pixels.get("right_wrist"),
            depth, depth_encoding, info, width, height,
        )
        typing_plane = self._typing_plane(
            left_landmarks,
            right_landmarks,
            raw_left_hand,
            raw_right_hand,
            depth,
            depth_encoding,
            info,
            width,
            height,
        )
        current_depths = [
            float(point[2])
            for point in (raw_left_hand, raw_right_hand)
            if point is not None
        ]
        current_hand_depth = (
            float(np.median(current_depths)) if current_depths else None
        )
        current_hands = [
            point
            for point in (raw_left_hand, raw_right_hand)
            if point is not None
        ]
        visible_count_msg = UInt8()
        visible_count_msg.data = len(current_hands)
        self._visible_hand_count_pub.publish(visible_count_msg)
        layout = hand_layout_features(left_landmarks, right_landmarks)
        if raw_left_hand is not None and raw_right_hand is not None:
            lateral_camera = raw_right_hand - raw_left_hand
            layout["hand_lateral_camera"] = [
                float(value) for value in lateral_camera
            ]
            layout["hand_depth_difference_m"] = float(lateral_camera[2])
        else:
            layout["hand_lateral_camera"] = None
            layout["hand_depth_difference_m"] = None
        layout_msg = String()
        layout_msg.data = json.dumps(layout)
        self._hand_layout_pub.publish(layout_msg)
        if current_hands:
            visual_target = np.mean(current_hands, axis=0)
            target_msg = PointStamped()
            target_msg.header.stamp = msg.header.stamp
            target_msg.header.frame_id = source_frame
            (
                target_msg.point.x,
                target_msg.point.y,
                target_msg.point.z,
            ) = map(float, visual_target)
            self._visual_target_pub.publish(target_msg)
        if current_hand_depth is not None:
            distance_msg = Float32()
            distance_msg.data = current_hand_depth
            self._hand_distance_pub.publish(distance_msg)
        fusion_time = time.monotonic()
        left_hand = self._remember_hand("left", raw_left_hand, fusion_time)
        right_hand = self._remember_hand("right", raw_right_hand, fusion_time)
        if left_hand is not None:
            points["left_hand"] = left_hand
        if right_hand is not None:
            points["right_hand"] = right_hand
        both_hands = left_hand is not None and right_hand is not None
        if both_hands:
            points["hand_midpoint"] = np.mean((left_hand, right_hand), axis=0)
        if typing_plane is not None:
            plane_midpoint, plane_rotation = typing_plane
            self._publish_typing_plane(
                msg, source_frame, plane_midpoint, plane_rotation
            )

        if pose is not None:
            visibility = float(
                np.mean(
                    [
                        pose[POSE.LEFT_SHOULDER].visibility,
                        pose[POSE.RIGHT_SHOULDER].visibility,
                        pose[POSE.LEFT_HIP].visibility,
                        pose[POSE.RIGHT_HIP].visibility,
                    ]
                )
            )
            candidate = self._classify_pose(pose)
        else:
            visibility = 0.9 if both_hands else 0.6
            candidate = "hands_only"
        stable_pose = self._update_stable_pose(candidate)
        both_hands_msg = Bool()
        both_hands_msg.data = both_hands
        self._both_hands_pub.publish(both_hands_msg)
        hand_depth = (
            float(points["hand_midpoint"][2]) if "hand_midpoint" in points else math.inf
        )
        # Conservative interpretation: when both hands are close to the tray camera,
        # forbid large repositioning even before tray-volume calibration is complete.
        hands_on = both_hands and hand_depth <= self._hands_on_depth

        user = UserState()
        user.header = msg.header
        user.person_detected = True
        user.person_count = 1
        user.confidence = visibility
        if pose is not None:
            shoulders = (pose[POSE.LEFT_SHOULDER], pose[POSE.RIGHT_SHOULDER])
            hips = (pose[POSE.LEFT_HIP], pose[POSE.RIGHT_HIP])
            user.center_x = float(np.mean([item.x for item in shoulders + hips]))
            user.center_y = float(np.mean([item.y for item in shoulders + hips]))
        elif left_landmarks is not None or right_landmarks is not None:
            visible_hands = [
                item
                for item in (left_landmarks, right_landmarks)
                if item is not None
            ]
            user.center_x = float(
                np.mean([item.landmark[0].x for item in visible_hands])
            )
            user.center_y = float(
                np.mean([item.landmark[0].y for item in visible_hands])
            )
        user.distance_m = (
            float(points["torso"][2])
            if "torso" in points
            else float(points["hand_midpoint"][2])
            if "hand_midpoint" in points
            else 0.0
        )
        if shoulder_l is not None and shoulder_r is not None:
            shoulder_width = max(0.02, abs(float(shoulder_r[0] - shoulder_l[0])))
            user.torso_yaw_deg = math.degrees(
                math.atan2(float(shoulder_r[2] - shoulder_l[2]), shoulder_width)
            )
        user.torso_pitch_deg = 0.0
        user.pose = stable_pose
        user.hands_on_keyboard = hands_on
        user.hands_clear = not hands_on
        user.stable_duration_sec = float(time.monotonic() - self._state_since)
        user.source = "mediapipe_holistic+independent_hands+orbbec_depth"
        self._user_pub.publish(user)

        base_ready = self._publish_points(msg, source_frame, points)
        self._publish_markers(msg, source_frame, points)
        self._publish_status(
            {
                "ok": True,
                "person": True,
                "pose_candidate": candidate,
                "pose_stable": stable_pose,
                "left_hand_3d": left_hand is not None,
                "right_hand_3d": right_hand is not None,
                "left_hand_current": raw_left_hand is not None,
                "right_hand_current": raw_right_hand is not None,
                "typing_plane_current": typing_plane is not None,
                "visible_hand_count": len(current_hands),
                "hand_layout": layout,
                "measured_hand_depth_m": (
                    round(current_hand_depth, 4)
                    if current_hand_depth is not None
                    else None
                ),
                "hands_on_keyboard": hands_on,
                "source_frame": source_frame,
                "base_frame": self._base_frame,
                "base_tf_ready": base_ready,
                "gesture_candidate": self._gesture_candidate,
                "gesture_hold_progress_sec": round(
                    max(
                        0.0,
                        time.monotonic() - self._gesture_candidate_since,
                    )
                    if self._timed_gesture_candidate is not None
                    else 0.0,
                    2,
                ),
                "last_gesture_command": self._last_gesture_command,
                "depth_m": round(float(user.distance_m), 3),
            }
        )
        self._draw_and_publish(
            msg, bgr, result, left_landmarks, right_landmarks
        )

    @staticmethod
    def _standalone_hands(result):
        output = {}
        landmarks = result.multi_hand_landmarks or []
        handedness = result.multi_handedness or []
        for index, hand in enumerate(landmarks):
            label = ""
            if index < len(handedness) and handedness[index].classification:
                label = handedness[index].classification[0].label.lower()
            if label in ("left", "right") and label not in output:
                output[label] = hand
        if len(landmarks) == 2 and len(output) < 2:
            ordered = sorted(landmarks, key=lambda hand: hand.landmark[0].x)
            output = {"left": ordered[0], "right": ordered[1]}
        elif len(landmarks) == 1 and not output:
            output["left"] = landmarks[0]
        return output.get("left"), output.get("right")

    def _pose_pixels(self, pose, width: int, height: int):
        mapping = {
            "left_shoulder": POSE.LEFT_SHOULDER,
            "right_shoulder": POSE.RIGHT_SHOULDER,
            "left_hip": POSE.LEFT_HIP,
            "right_hip": POSE.RIGHT_HIP,
            "left_wrist": POSE.LEFT_WRIST,
            "right_wrist": POSE.RIGHT_WRIST,
        }
        output = {}
        for name, index in mapping.items():
            landmark = pose[index]
            if landmark.visibility >= self._min_visibility:
                output[name] = (landmark.x * width, landmark.y * height)
        return output

    def _point3d(self, pixel, depth, encoding, info) -> Optional[np.ndarray]:
        if pixel is None:
            return None
        u, v = pixel
        value = median_depth_m(
            depth, encoding, round(u), round(v), self._radius,
            self._min_depth, self._max_depth,
        )
        return deproject(info, u, v, value) if value is not None else None

    def _remember_hand(
        self, name: str, point: Optional[np.ndarray], now: float
    ) -> Optional[np.ndarray]:
        """Bridge brief left/right detector flicker without inventing a person.

        This is only called after at least one hand was detected in the current
        image.  A missing mate may be reused for at most ``hand_memory_sec``;
        the controller's own target timeout remains the final stop condition.
        """
        if point is not None:
            self._last_hand_points[name] = (point.copy(), now)
            return point
        previous = self._last_hand_points.get(name)
        if previous is None or now - previous[1] > self._hand_memory_sec:
            return None
        return previous[0].copy()

    def _hand_center(
        self, hand_landmarks, fallback_wrist, depth, encoding, info, width, height
    ) -> Optional[np.ndarray]:
        pixels = []
        if hand_landmarks is not None:
            # Palm perimeter and centre MCPs give a robust depth cluster.
            for index in (0, 1, 5, 9, 13, 17):
                landmark = hand_landmarks.landmark[index]
                pixels.append((landmark.x * width, landmark.y * height))
        elif fallback_wrist is not None:
            pixels.append(fallback_wrist)
        if not pixels:
            return None
        depth_m = robust_landmark_depth_m(
            depth,
            encoding,
            pixels,
            max(2, self._radius // 2),
            self._min_depth,
            self._max_depth,
            minimum_samples=2 if len(pixels) > 1 else 1,
        )
        if depth_m is None:
            return None
        palm_pixel = np.median(np.asarray(pixels, dtype=float), axis=0)
        return deproject(info, palm_pixel[0], palm_pixel[1], depth_m)

    def _hand_forward(
        self, hand_landmarks, depth, encoding, info, width, height
    ) -> Optional[np.ndarray]:
        if hand_landmarks is None:
            return None
        points = []
        for index in (0, 9):
            landmark = hand_landmarks.landmark[index]
            u, v = landmark.x * width, landmark.y * height
            depth_m = median_depth_m(
                depth,
                encoding,
                round(u),
                round(v),
                max(2, self._radius // 2),
                self._min_depth,
                self._max_depth,
            )
            if depth_m is None:
                return None
            points.append(deproject(info, u, v, depth_m))
        forward = points[1] - points[0]
        if not np.all(np.isfinite(forward)) or np.linalg.norm(forward) < 0.015:
            return None
        return forward / np.linalg.norm(forward)

    def _typing_plane(
        self,
        left_landmarks,
        right_landmarks,
        left_center,
        right_center,
        depth,
        encoding,
        info,
        width,
        height,
    ):
        if left_center is None or right_center is None:
            return None
        left_forward = self._hand_forward(
            left_landmarks, depth, encoding, info, width, height
        )
        right_forward = self._hand_forward(
            right_landmarks, depth, encoding, info, width, height
        )
        if left_forward is None or right_forward is None:
            return None
        try:
            rotation = typing_plane_rotation(
                left_center, right_center, left_forward, right_forward
            )
        except ValueError:
            return None
        return np.mean((left_center, right_center), axis=0), rotation

    def _publish_typing_plane(
        self,
        msg: Image,
        source_frame: str,
        midpoint: np.ndarray,
        rotation: np.ndarray,
    ) -> None:
        quaternion = matrix_quaternion(rotation)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = source_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
            float, midpoint
        )
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion
        self._typing_plane_pub.publish(pose)
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = "human_typing_plane"
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _classify_pose(self, pose: Sequence) -> str:
        center_x = float(
            np.mean(
                [
                    pose[POSE.LEFT_SHOULDER].x,
                    pose[POSE.RIGHT_SHOULDER].x,
                    pose[POSE.LEFT_HIP].x,
                    pose[POSE.RIGHT_HIP].x,
                ]
            )
        )
        if center_x < 0.5 - self._side_threshold:
            return "left"
        if center_x > 0.5 + self._side_threshold:
            return "right"
        hip_y = 0.5 * (pose[POSE.LEFT_HIP].y + pose[POSE.RIGHT_HIP].y)
        knee_y = 0.5 * (pose[POSE.LEFT_KNEE].y + pose[POSE.RIGHT_KNEE].y)
        knee_visibility = min(
            pose[POSE.LEFT_KNEE].visibility, pose[POSE.RIGHT_KNEE].visibility
        )
        if knee_visibility >= self._min_visibility and knee_y - hip_y > self._standing_threshold:
            return "standing"
        return "seated"

    def _update_stable_pose(self, candidate: str) -> str:
        self._state_votes.append(candidate)
        if (
            len(self._state_votes) == self._state_votes.maxlen
            and all(value == candidate for value in self._state_votes)
            and candidate != self._stable_pose
        ):
            self._stable_pose = candidate
            self._state_since = time.monotonic()
        return self._stable_pose

    def _publish_points(
        self, msg: Image, source_frame: str, points: Dict[str, np.ndarray]
    ) -> bool:
        transform = None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, source_frame, Time()
            )
        except Exception:
            pass
        for name, point in points.items():
            stamped = PointStamped()
            stamped.header.stamp = msg.header.stamp
            stamped.header.frame_id = source_frame
            stamped.point.x, stamped.point.y, stamped.point.z = map(float, point)
            self._point_pubs[name].publish(stamped)
            tf_msg = TransformStamped()
            tf_msg.header = stamped.header
            tf_msg.child_frame_id = f"human_{name}"
            tf_msg.transform.translation.x = stamped.point.x
            tf_msg.transform.translation.y = stamped.point.y
            tf_msg.transform.translation.z = stamped.point.z
            tf_msg.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(tf_msg)
            if transform is not None:
                q = transform.transform.rotation
                rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
                translation = np.array(
                    [
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ]
                )
                base_point = rotation @ point + translation
                base_msg = PointStamped()
                base_msg.header.stamp = msg.header.stamp
                base_msg.header.frame_id = self._base_frame
                base_msg.point.x, base_msg.point.y, base_msg.point.z = map(
                    float, base_point
                )
                self._base_point_pubs[name].publish(base_msg)
        return transform is not None

    def _publish_markers(
        self, msg: Image, source_frame: str, points: Dict[str, np.ndarray]
    ) -> None:
        colors = {
            "torso": (0.15, 0.55, 1.0),
            "left_hand": (0.1, 1.0, 0.2),
            "right_hand": (1.0, 0.3, 0.1),
            "hand_midpoint": (1.0, 1.0, 0.0),
        }
        array = MarkerArray()
        for marker_id, (name, point) in enumerate(points.items()):
            marker = Marker()
            marker.header.stamp = msg.header.stamp
            marker.header.frame_id = source_frame
            marker.ns = "human_depth_landmarks"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(
                float, point
            )
            marker.pose.orientation.w = 1.0
            size = 0.075 if name == "torso" else 0.045
            marker.scale.x = marker.scale.y = marker.scale.z = size
            red, green, blue = colors[name]
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 400_000_000
            array.markers.append(marker)
        self._marker_pub.publish(array)

    def _publish_absent(self, msg: Image, reason: str) -> None:
        self._update_gesture(None)
        both_hands = Bool()
        both_hands.data = False
        self._both_hands_pub.publish(both_hands)
        visible_count = UInt8()
        visible_count.data = 0
        self._visible_hand_count_pub.publish(visible_count)
        layout = String()
        layout.data = json.dumps(hand_layout_features(None, None))
        self._hand_layout_pub.publish(layout)
        stable = self._update_stable_pose("absent")
        user = UserState()
        user.header = msg.header
        user.pose = stable
        user.hands_clear = True
        user.stable_duration_sec = float(time.monotonic() - self._state_since)
        user.source = "mediapipe_holistic+orbbec_registered_depth"
        self._user_pub.publish(user)
        self._publish_status({"ok": reason == "no_person", "person": False, "reason": reason})

    def _update_gesture(self, candidate: Optional[str]) -> None:
        now = time.monotonic()
        self._gesture_candidate = candidate
        if candidate != self._last_logged_gesture_candidate:
            self._last_logged_gesture_candidate = candidate
            if candidate is not None:
                self.get_logger().info(
                    f"GESTURE CANDIDATE: {candidate} "
                    f"(hold continuously for {self._gesture_hold_sec:.1f} s)"
                )
        if candidate not in ("one", "two"):
            self._timed_gesture_candidate = None
            self._gesture_candidate_since = 0.0
            self._gesture_neutral_count += 1
            if self._gesture_neutral_count >= self._gesture_neutral_required:
                self._gesture_armed = True
            return
        self._gesture_neutral_count = 0
        command = "start" if candidate == "one" else "home"
        if not self._gesture_armed:
            # Permit a direct 1 -> 2 or 2 -> 1 transition. Repeating the same
            # command still requires a brief neutral hand to prevent flooding.
            if command == self._last_gesture_command:
                self._timed_gesture_candidate = None
                self._gesture_candidate_since = 0.0
                return
            self._gesture_armed = True
            self._timed_gesture_candidate = None
            self._gesture_candidate_since = 0.0
        if self._timed_gesture_candidate != candidate:
            self._timed_gesture_candidate = candidate
            self._gesture_candidate_since = now
            return
        if now - self._gesture_candidate_since < self._gesture_hold_sec:
            return
        message = String()
        message.data = command
        self._gesture_pub.publish(message)
        self._last_gesture_command = command
        self._gesture_armed = False
        self._timed_gesture_candidate = None
        self._gesture_candidate_since = 0.0
        self.get_logger().warning(
            f"GESTURE COMMAND: {candidate} -> {command}"
        )

    def _draw_and_publish(
        self, msg: Image, bgr: np.ndarray, result, left_landmarks, right_landmarks
    ) -> None:
        if not self._publish_debug:
            return
        drawing = mp.solutions.drawing_utils
        drawing.draw_landmarks(
            bgr, result.pose_landmarks, mp.solutions.holistic.POSE_CONNECTIONS
        )
        drawing.draw_landmarks(
            bgr, left_landmarks, mp.solutions.holistic.HAND_CONNECTIONS
        )
        drawing.draw_landmarks(
            bgr, right_landmarks, mp.solutions.holistic.HAND_CONNECTIONS
        )
        height, width = bgr.shape[:2]
        target_u = width // 2
        target_v = int(round(0.40 * height))
        if self._camera_info is not None:
            target_u = int(round(float(self._camera_info.k[2])))
            target_v = int(
                round(
                    float(self._camera_info.k[5])
                    + float(self._camera_info.k[4])
                    * math.tan(self._tracking_vertical_offset)
                )
            )
        target_u = int(np.clip(target_u, 0, width - 1))
        target_v = int(np.clip(target_v, 0, height - 1))
        cv2.drawMarker(
            bgr,
            (target_u, target_v),
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=28,
            thickness=2,
        )
        gesture_hold_progress = (
            max(0.0, time.monotonic() - self._gesture_candidate_since)
            if self._timed_gesture_candidate is not None
            else 0.0
        )
        cv2.putText(
            bgr,
            (
                f"gesture={self._gesture_candidate or '-'} "
                f"hold={gesture_hold_progress:.1f}/"
                f"{self._gesture_hold_sec:.1f}s "
                f"last={self._last_gesture_command or '-'}"
            ),
            (16, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        debug = self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        debug.header = msg.header
        self._debug_pub.publish(debug)

    def _publish_status(self, payload) -> None:
        signature = (
            payload.get("ok"),
            payload.get("person"),
            payload.get("reason"),
            payload.get("left_hand_3d"),
            payload.get("right_hand_3d"),
            payload.get("typing_plane_current"),
            payload.get("visible_hand_count"),
            payload.get("hands_on_keyboard"),
            payload.get("base_tf_ready"),
        )
        if signature != self._last_status_signature:
            self._last_status_signature = signature
            self.get_logger().info(
                "HUMAN TRACKING: "
                f"person={payload.get('person', False)}, "
                f"left3d={payload.get('left_hand_3d', False)}, "
                f"right3d={payload.get('right_hand_3d', False)}, "
                f"typing_plane={payload.get('typing_plane_current', False)}, "
                f"visible_hands={payload.get('visible_hand_count', 0)}, "
                f"hands_on_keyboard={payload.get('hands_on_keyboard', False)}, "
                f"base_tf={payload.get('base_tf_ready', False)}, "
                f"reason={payload.get('reason', 'ok')}"
            )
        status = String()
        status.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(status)

    def destroy_node(self):
        if self._holistic is not None:
            self._holistic.close()
        self._hands.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MediaPipeBodyHandNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # ROS 2 Humble can raise from take_message while launch is tearing down
        # the context. Preserve real runtime failures but make Ctrl-C clean.
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
