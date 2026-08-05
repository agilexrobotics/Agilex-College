"""ROS 2 node for YOLOv5-Face landmarks on Orbbec RGB/depth streams.

The node keeps camera-frame latency low by queueing only the newest RGB frame,
runs ONNX inference in a worker thread, publishes 2D detections/landmarks, and
optionally projects the selected face into 3D for pose and TF output.
"""

import math
import os
import queue
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, Header, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


def _resolve_qos(use_sensor_data_qos: bool) -> QoSProfile:
    """Choose QoS compatible with camera streams or reliable downstream consumers."""
    if use_sensor_data_qos:
        return QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
    return QoSProfile(
        depth=5,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
    )


def _to_tuple_color(value: Sequence[int], default: Tuple[int, int, int]) -> Tuple[int, int, int]:
    """Convert a ROS parameter sequence into an OpenCV BGR color tuple."""
    try:
        if len(value) >= 3:
            return int(value[0]), int(value[1]), int(value[2])
    except Exception:
        pass
    return default


def _letterbox(image: np.ndarray, target_shape=(640, 640), color=(114, 114, 114)) -> np.ndarray:
    """Resize with unchanged aspect ratio and pad to the model input size."""
    height, width = image.shape[:2]
    scale = min(target_shape[0] / height, target_shape[1] / width)
    new_size = (int(width * scale), int(height * scale))
    resized = cv2.resize(image, new_size, interpolation=cv2.INTER_LINEAR)
    pad_x = (target_shape[1] - new_size[0]) / 2
    pad_y = (target_shape[0] - new_size[1]) / 2
    top = int(pad_y)
    bottom = int(target_shape[0] - new_size[1] - top)
    left = int(pad_x)
    right = int(target_shape[1] - new_size[0] - left)
    return cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)


def _scale_boxes(img1_shape, boxes: np.ndarray, img0_shape) -> np.ndarray:
    """Map bounding boxes from letterboxed model coordinates back to the RGB image."""
    gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
    pad_x = (img1_shape[1] - img0_shape[1] * gain) / 2
    pad_y = (img1_shape[0] - img0_shape[0] * gain) / 2
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / gain
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / gain
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, img0_shape[1] - 1)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, img0_shape[0] - 1)
    return boxes


def _scale_landmarks(img1_shape, landmarks: np.ndarray, img0_shape) -> np.ndarray:
    """Map five facial landmark points from model coordinates back to image pixels."""
    gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
    pad_x = (img1_shape[1] - img0_shape[1] * gain) / 2
    pad_y = (img1_shape[0] - img0_shape[0] * gain) / 2
    landmarks[:, 0::2] = (landmarks[:, 0::2] - pad_x) / gain
    landmarks[:, 1::2] = (landmarks[:, 1::2] - pad_y) / gain
    landmarks[:, 0::2] = landmarks[:, 0::2].clip(0, img0_shape[1] - 1)
    landmarks[:, 1::2] = landmarks[:, 1::2].clip(0, img0_shape[0] - 1)
    return landmarks


def _xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    """Convert YOLO center-size boxes to corner boxes for NMS and publishing."""
    out = boxes.copy()
    out[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
    out[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
    out[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
    out[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
    return out


def _normalize(vector: np.ndarray, min_norm: float = 1e-9) -> Optional[np.ndarray]:
    """Return a unit vector, or None when the input is too small to be stable."""
    norm = float(np.linalg.norm(vector))
    if norm < min_norm:
        return None
    return vector / norm


def _quat_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert a ROS quaternion into a 3x3 rotation matrix."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _rotation_matrix_to_quat(matrix: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized ROS quaternion tuple."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2.0
        if s < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        qw = (matrix[2, 1] - matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (matrix[0, 1] + matrix[1, 0]) / s
        qz = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] + matrix[2, 2])) * 2.0
        if s < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        qw = (matrix[0, 2] - matrix[2, 0]) / s
        qx = (matrix[0, 1] + matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])) * 2.0
        if s < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        qw = (matrix[1, 0] - matrix[0, 1]) / s
        qx = (matrix[0, 2] + matrix[2, 0]) / s
        qy = (matrix[1, 2] + matrix[2, 1]) / s
        qz = 0.25 * s

    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    quat_norm = float(np.linalg.norm(quat))
    if quat_norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    quat /= quat_norm
    return float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float, max_det: int) -> np.ndarray:
    """Run class-agnostic non-maximum suppression on face boxes."""
    if boxes.size == 0:
        return np.array([], dtype=np.int64)

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if max_det > 0 and len(keep) >= max_det:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[order[1:]] - inter
        iou = np.divide(inter, union, out=np.zeros_like(inter), where=union > 0)
        order = order[np.where(iou <= iou_threshold)[0] + 1]

    return np.array(keep, dtype=np.int64)


@dataclass
class FaceLandmarkRecord:
    """One detected face in RGB pixel coordinates."""

    confidence: float
    left: float
    top: float
    right: float
    bottom: float
    left_eye: Tuple[float, float]
    right_eye: Tuple[float, float]
    nose: Tuple[float, float]
    left_mouth: Tuple[float, float]
    right_mouth: Tuple[float, float]

    @property
    def center_x(self) -> float:
        return (self.left + self.right) * 0.5

    @property
    def center_y(self) -> float:
        return (self.top + self.bottom) * 0.5

    @property
    def width(self) -> float:
        return self.right - self.left

    @property
    def height(self) -> float:
        return self.bottom - self.top

    @property
    def area(self) -> float:
        return max(0.0, self.width) * max(0.0, self.height)

    @property
    def mouth_center(self) -> Tuple[float, float]:
        return (
            (self.left_mouth[0] + self.right_mouth[0]) * 0.5,
            (self.left_mouth[1] + self.right_mouth[1]) * 0.5,
        )


@dataclass
class ProjectedFaceLandmarks:
    """Selected face landmarks after depth lookup and camera deprojection."""

    header: Header
    points: List[Point]


class OrbbecFaceLandmarkNode(Node):
    """Subscribe to Orbbec images and publish face landmarks, detections, pose, and TF."""

    def __init__(self) -> None:
        super().__init__("orbbec_face_landmark")

        # Topic and frame parameters define the ROS graph interface.
        self.color_image_topic = self.declare_parameter("color_image_topic", "/camera/color/image_raw").value
        self.color_camera_info_topic = self.declare_parameter(
            "color_camera_info_topic", "/camera/color/camera_info"
        ).value
        self.aligned_depth_topic = self.declare_parameter(
            "aligned_depth_topic", "/camera/depth/image_raw"
        ).value
        self.landmarks_2d_topic = self.declare_parameter("landmarks_2d_topic", "/face_landmarks_2d").value
        self.landmarks_3d_topic = self.declare_parameter("landmarks_3d_topic", "/face_landmarks_3d").value
        self.face_pose_topic = self.declare_parameter("face_pose_topic", "/face_pose").value
        self.detections_topic = self.declare_parameter("detections_topic", "/face_detections_2d").value
        self.debug_image_topic = self.declare_parameter("debug_image_topic", "/face_landmarks_debug_image").value
        self.status_topic = self.declare_parameter("status_topic", "/orbbec_face_landmark/status").value
        self.target_frame = self.declare_parameter("target_frame", "camera_link").value
        self.face_frame_id = self.declare_parameter("face_frame_id", "face").value

        # Inference parameters mirror the exported YOLOv5-Face ONNX model.
        self.model_path = self.declare_parameter(
            "model_path", os.environ.get("SCREEN_FACE_MODEL", "")
        ).value
        self.inference_backend = self.declare_parameter("inference_backend", "auto").value
        self.device = self.declare_parameter("device", "cpu").value
        self.img_size = int(self.declare_parameter("img_size", 640).value)
        self.conf_thres = float(self.declare_parameter("confidence_threshold", 0.6).value)
        self.iou_thres = float(self.declare_parameter("iou_threshold", 0.5).value)
        self.max_det = int(self.declare_parameter("max_det", 20).value)
        self.max_fps = float(self.declare_parameter("max_fps", 30.0).value)
        self.profile_inference = bool(self.declare_parameter("profile_inference", False).value)
        self.use_sensor_data_qos = bool(self.declare_parameter("use_sensor_data_qos", True).value)
        self.select_largest_face = bool(self.declare_parameter("select_largest_face", True).value)

        # Publishing and projection controls keep the node usable in 2D-only setups.
        self.publish_debug_image = bool(self.declare_parameter("publish_debug_image", True).value)
        self.publish_empty_2d = bool(self.declare_parameter("publish_empty_2d", False).value)
        self.publish_empty_detections = bool(
            self.declare_parameter("publish_empty_detections", True).value
        )
        self.enable_depth_projection = bool(
            self.declare_parameter("enable_depth_projection", True).value
        )
        self.depth_window_radius = int(self.declare_parameter("depth_window_radius", 3).value)
        self.min_depth_m = float(self.declare_parameter("min_depth_m", 0.2).value)
        self.max_depth_m = float(self.declare_parameter("max_depth_m", 5.0).value)

        # Temporal filters smooth jitter from both detector pixels and sampled depth.
        self.enable_temporal_filter = bool(
            self.declare_parameter("enable_temporal_filter", True).value
        )
        self.landmark_filter_alpha = float(
            self.declare_parameter("landmark_filter_alpha", 0.35).value
        )
        self.depth_filter_alpha = float(
            self.declare_parameter("depth_filter_alpha", 0.25).value
        )
        self.min_eye_distance_m = float(self.declare_parameter("min_eye_distance_m", 0.02).value)
        self.min_plane_normal_norm = float(self.declare_parameter("min_plane_normal_norm", 1e-4).value)
        self.publish_face_tf = bool(self.declare_parameter("publish_face_tf", True).value)
        self.debug_thickness = int(self.declare_parameter("debug_thickness", 2).value)
        self.debug_font_scale = float(self.declare_parameter("debug_font_scale", 0.6).value)
        self.debug_color_box = _to_tuple_color(
            self.declare_parameter("debug_color_box", [0, 255, 0]).value,
            (0, 255, 0),
        )

        self._validate_environment()
        self.bridge = CvBridge()
        self.qos = _resolve_qos(self.use_sensor_data_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.image_sub = self.create_subscription(
            Image, self.color_image_topic, self._image_callback, self.qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.color_camera_info_topic, self._camera_info_callback, self.qos
        )
        self.depth_sub = self.create_subscription(
            Image, self.aligned_depth_topic, self._depth_callback, self.qos
        )
        self.landmarks_2d_pub = self.create_publisher(Float32MultiArray, self.landmarks_2d_topic, 10)
        self.landmarks_3d_pub = self.create_publisher(Float32MultiArray, self.landmarks_3d_topic, 10)
        self.face_pose_pub = self.create_publisher(PoseStamped, self.face_pose_topic, 10)
        self.detections_pub = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.camera_info_msg: Optional[CameraInfo] = None
        self.depth_msg: Optional[Image] = None
        self.depth_image: Optional[np.ndarray] = None
        self._model_input_name: Optional[str] = None
        self._model_output_name: Optional[str] = None
        self._active_inference_backend = "opencv"
        self._active_inference_device = "cpu"
        self._frame_queue: "queue.Queue[Optional[Tuple[Header, np.ndarray]]]" = queue.Queue(maxsize=1)
        self._worker_stop = threading.Event()

        # The worker owns inference; callbacks only enqueue frames to avoid blocking ROS.
        self._lock = threading.Lock()
        self._busy = False
        self._last_infer_time = 0.0
        self._last_status_time = 0.0
        self._profile_count = 0
        self._filtered_face: Optional[FaceLandmarkRecord] = None
        self._filtered_points_3d: Optional[np.ndarray] = None
        self.model = self._load_onnx_model()

        self.get_logger().info("orbbec_face_landmark ready")
        self.get_logger().info(f"color_image_topic={self.color_image_topic}")
        self.get_logger().info(f"aligned_depth_topic={self.aligned_depth_topic}")
        self.get_logger().info(f"color_camera_info_topic={self.color_camera_info_topic}")
        self.get_logger().info(f"face_pose_topic={self.face_pose_topic}")
        self.get_logger().info(f"target_frame={self.target_frame}")
        self.get_logger().info(
            f"temporal_filter={self.enable_temporal_filter}, "
            f"landmark_alpha={self.landmark_filter_alpha:.3f}, "
            f"depth_alpha={self.depth_filter_alpha:.3f}"
        )
        self.get_logger().info(
            f"inference_backend={self._active_inference_backend}, "
            f"inference_device={self._active_inference_device}"
        )
        self.get_logger().info(f"model_path={self.model_path}")

        self._worker_thread = threading.Thread(
            target=self._inference_worker,
            name="face_inference_worker",
            daemon=True,
        )
        self._worker_thread.start()

    def _validate_environment(self) -> None:
        """Fail early when the ONNX model path is missing."""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(
                f"YOLOv5-Face ONNX model file not found: {self.model_path}"
            )

    def _load_onnx_model(self):
        """Prefer ONNX Runtime for GPU/CPU inference and fall back to OpenCV DNN."""
        if self.img_size != 640:
            raise ValueError("YOLOv5-Face ONNX weights in this package support img_size=640")

        requested_backend = str(self.inference_backend).lower()
        requested_device = str(self.device).lower()
        should_try_onnxruntime = requested_backend in ("auto", "onnxruntime", "ort")
        if requested_device.startswith("cuda"):
            should_try_onnxruntime = True

        if should_try_onnxruntime:
            onnxruntime_model = self._load_onnxruntime_model(requested_device)
            if onnxruntime_model is not None:
                return onnxruntime_model
            if requested_backend in ("onnxruntime", "ort") or requested_device.startswith("cuda"):
                # Keep the node alive on machines without CUDA provider support.
                self.get_logger().warn(
                    "ONNX Runtime GPU is unavailable; falling back to OpenCV CPU inference"
                )

        net = cv2.dnn.readNetFromONNX(self.model_path)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._active_inference_backend = "opencv"
        self._active_inference_device = "cpu"
        return net

    def _load_onnxruntime_model(self, requested_device: str):
        """Create an ONNX Runtime session and record the activated execution provider."""
        try:
            import onnxruntime as ort
        except ImportError as exc:
            self.get_logger().warn(f"onnxruntime is not installed: {exc}")
            return None

        try:
            # onnxruntime-gpu can load CUDA/cuDNN libraries installed in the Python
            # environment through this helper.
            if hasattr(ort, "preload_dlls"):
                ort.preload_dlls()

            session_options = ort.SessionOptions()
            session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

            use_cuda = requested_device.startswith("cuda")
            if use_cuda:
                # Provider availability is checked before session creation so the log
                # points at the Python/CUDA environment instead of model loading.
                available = ort.get_available_providers()
                if "CUDAExecutionProvider" not in available:
                    self.get_logger().warn(
                        f"CUDAExecutionProvider is not available: {available}"
                    )
                    return None
                device_id = 0
                if ":" in requested_device:
                    try:
                        device_id = int(requested_device.split(":", 1)[1])
                    except ValueError:
                        self.get_logger().warn(
                            f"invalid CUDA device '{requested_device}', using device 0"
                        )
                providers = [
                    ("CUDAExecutionProvider", {"device_id": device_id}),
                    "CPUExecutionProvider",
                ]
            else:
                providers = ["CPUExecutionProvider"]

            session = ort.InferenceSession(
                self.model_path,
                sess_options=session_options,
                providers=providers,
            )
            active_providers = session.get_providers()
            if use_cuda and not active_providers:
                self.get_logger().warn("ONNX Runtime created no execution provider")
                return None
            if use_cuda and active_providers[0] != "CUDAExecutionProvider":
                self.get_logger().warn(
                    f"ONNX Runtime did not activate CUDA: {active_providers}"
                )
                return None

            self._model_input_name = session.get_inputs()[0].name
            self._model_output_name = session.get_outputs()[0].name
            self._active_inference_backend = "onnxruntime"
            self._active_inference_device = "cuda" if use_cuda else "cpu"
            self.get_logger().info(
                f"Using ONNX Runtime providers={active_providers}"
            )
            return session
        except Exception as exc:
            self.get_logger().warn(f"ONNX Runtime initialization failed: {exc}")
            return None

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Store camera intrinsics used later to deproject depth pixels."""
        self.camera_info_msg = msg

    def _depth_callback(self, msg: Image) -> None:
        """Cache the latest aligned depth image for the next selected face."""
        if not self.enable_depth_projection:
            return
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_msg = msg
        except Exception as exc:
            self._publish_status("error", f"depth conversion failed: {exc}")

    def _image_callback(self, msg: Image) -> None:
        """Convert RGB frames and enqueue the newest one for asynchronous inference."""
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._publish_status("error", str(exc))
            self.get_logger().error(f"Color image conversion failed: {exc}")
            return

        item = (msg.header, frame_bgr)
        try:
            # Keep only the newest frame. Processing an old frame would increase
            # latency without improving the pose output.
            if self._frame_queue.full():
                self._frame_queue.get_nowait()
            self._frame_queue.put_nowait(item)
        except queue.Full:
            pass

    def _inference_worker(self) -> None:
        """Drain the single-frame queue and run inference outside ROS callbacks."""
        while not self._worker_stop.is_set():
            try:
                item = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if item is None:
                break

            now = time.time()
            if self.max_fps > 0 and self._last_infer_time > 0:
                # Drop frames that arrive faster than the configured inference budget.
                if now - self._last_infer_time < 1.0 / self.max_fps:
                    continue

            header, frame_bgr = item
            callback_start = time.perf_counter()
            try:
                inference_start = time.perf_counter()
                faces, debug_bgr = self._run_inference(frame_bgr)
                inference_end = time.perf_counter()
                self._publish_results(header, faces, debug_bgr)
                publish_end = time.perf_counter()
                self._last_infer_time = time.time()
                if self.profile_inference:
                    self._profile_count += 1
                    if self._profile_count % 10 == 0:
                        self.get_logger().info(
                            "profile: "
                            f"worker_ms={(publish_end - callback_start) * 1000.0:.2f}, "
                            f"inference_ms={(inference_end - inference_start) * 1000.0:.2f}, "
                            f"publish_ms={(publish_end - inference_end) * 1000.0:.2f}, "
                            f"faces={len(faces)}"
                        )
            except Exception as exc:
                self._publish_status("error", str(exc))
                self.get_logger().error(f"Face landmark inference failed: {exc}")

    def destroy_node(self):
        """Stop the inference worker before letting rclpy tear down publishers."""
        self._worker_stop.set()
        try:
            self._frame_queue.put_nowait(None)
        except queue.Full:
            try:
                self._frame_queue.get_nowait()
                self._frame_queue.put_nowait(None)
            except queue.Empty:
                pass
        if hasattr(self, "_worker_thread") and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        return super().destroy_node()

    def _run_inference(self, frame_bgr: np.ndarray):
        """Run model preprocessing, inference, YOLO postprocessing, and debug drawing."""
        im0 = frame_bgr.copy()
        input_shape = (self.img_size, self.img_size)
        resized = _letterbox(im0, input_shape)
        # YOLOv5-Face expects normalized RGB NCHW input.
        img = resized[:, :, ::-1].astype(np.float32) / 255.0
        img = np.ascontiguousarray(img.transpose(2, 0, 1)[None])

        if self._active_inference_backend == "onnxruntime":
            pred = self.model.run(
                [self._model_output_name],
                {self._model_input_name: img},
            )[0]
        else:
            self.model.setInput(img)
            pred = self.model.forward()
        if pred.ndim == 3:
            pred = pred[0]

        pred = pred[pred[:, 4] >= self.conf_thres]
        faces: List[FaceLandmarkRecord] = []
        if pred.size:
            # Output layout: cx, cy, w, h, confidence, then five (x, y) landmarks.
            boxes = _xywh_to_xyxy(pred[:, :4])
            scores = pred[:, 4]
            landmarks = pred[:, 5:15].copy()
            keep = _nms(boxes, scores, self.iou_thres, self.max_det)
            boxes = _scale_boxes(input_shape, boxes[keep], im0.shape).round()
            landmarks = _scale_landmarks(input_shape, landmarks[keep], im0.shape).round()
            scores = scores[keep]

            for box, conf, lm in zip(boxes.tolist(), scores.tolist(), landmarks.tolist()):
                x1, y1, x2, y2 = box
                faces.append(
                    FaceLandmarkRecord(
                        confidence=float(conf),
                        left=float(x1),
                        top=float(y1),
                        right=float(x2),
                        bottom=float(y2),
                        left_eye=(float(lm[0]), float(lm[1])),
                        right_eye=(float(lm[2]), float(lm[3])),
                        nose=(float(lm[4]), float(lm[5])),
                        left_mouth=(float(lm[6]), float(lm[7])),
                        right_mouth=(float(lm[8]), float(lm[9])),
                    )
                )

        if self.select_largest_face and len(faces) > 1:
            faces.sort(key=lambda face: face.area, reverse=True)

        debug_bgr = self._draw_debug_image(im0, faces) if self.publish_debug_image else im0
        return faces, debug_bgr

    def _draw_debug_image(self, image: np.ndarray, faces: List[FaceLandmarkRecord]) -> np.ndarray:
        """Overlay detections and the four published landmark points on the image."""
        debug = image.copy()
        point_specs = [
            ("L", (255, 0, 0), lambda face: face.left_eye),
            ("R", (0, 255, 0), lambda face: face.right_eye),
            ("N", (0, 0, 255), lambda face: face.nose),
            ("M", (255, 255, 0), lambda face: face.mouth_center),
        ]
        for idx, face in enumerate(faces):
            color = self.debug_color_box if idx == 0 else (160, 160, 160)
            pt1 = (int(round(face.left)), int(round(face.top)))
            pt2 = (int(round(face.right)), int(round(face.bottom)))
            cv2.rectangle(debug, pt1, pt2, color, self.debug_thickness)
            cv2.putText(
                debug,
                f"face {face.confidence:.2f}",
                (pt1[0], max(15, pt1[1] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.debug_font_scale,
                color,
                max(1, self.debug_thickness - 1),
                cv2.LINE_AA,
            )
            for name, point_color, getter in point_specs:
                point = getter(face)
                center = (int(round(point[0])), int(round(point[1])))
                cv2.circle(debug, center, max(2, self.debug_thickness + 1), point_color, -1)
                cv2.putText(
                    debug,
                    name,
                    (center[0] + 4, center[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    point_color,
                    1,
                    cv2.LINE_AA,
                )
        return debug

    def _reset_temporal_filter(self) -> None:
        """Clear smoothing state when tracking is lost or no face is published."""
        self._filtered_face = None
        self._filtered_points_3d = None

    @staticmethod
    def _ema(previous: float, current: float, alpha: float) -> float:
        """Single-sample exponential moving average."""
        return previous + alpha * (current - previous)

    def _filter_face_landmarks(self, face: FaceLandmarkRecord) -> FaceLandmarkRecord:
        """Smooth 2D box and landmark coordinates frame-to-frame."""
        if not self.enable_temporal_filter:
            return face

        alpha = min(1.0, max(0.0, self.landmark_filter_alpha))
        previous = self._filtered_face
        if previous is None or alpha >= 1.0:
            self._filtered_face = face
            return face

        def smooth_point(
            old_point: Tuple[float, float],
            new_point: Tuple[float, float],
        ) -> Tuple[float, float]:
            return (
                self._ema(old_point[0], new_point[0], alpha),
                self._ema(old_point[1], new_point[1], alpha),
            )

        filtered = FaceLandmarkRecord(
            confidence=face.confidence,
            left=self._ema(previous.left, face.left, alpha),
            top=self._ema(previous.top, face.top, alpha),
            right=self._ema(previous.right, face.right, alpha),
            bottom=self._ema(previous.bottom, face.bottom, alpha),
            left_eye=smooth_point(previous.left_eye, face.left_eye),
            right_eye=smooth_point(previous.right_eye, face.right_eye),
            nose=smooth_point(previous.nose, face.nose),
            left_mouth=smooth_point(previous.left_mouth, face.left_mouth),
            right_mouth=smooth_point(previous.right_mouth, face.right_mouth),
        )
        self._filtered_face = filtered
        return filtered

    def _filter_3d_points(self, points: List[Point]) -> List[Point]:
        """Smooth deprojected 3D landmark points frame-to-frame."""
        current = np.array(
            [[point.x, point.y, point.z] for point in points],
            dtype=np.float64,
        )
        if not self.enable_temporal_filter:
            return points

        alpha = min(1.0, max(0.0, self.depth_filter_alpha))
        if self._filtered_points_3d is None or self._filtered_points_3d.shape != current.shape:
            self._filtered_points_3d = current
        elif alpha >= 1.0:
            self._filtered_points_3d = current
        else:
            self._filtered_points_3d += alpha * (current - self._filtered_points_3d)

        return [
            Point(
                x=float(point[0]),
                y=float(point[1]),
                z=float(point[2]),
            )
            for point in self._filtered_points_3d
        ]

    def _publish_results(self, header: Header, faces: List[FaceLandmarkRecord], debug_bgr: np.ndarray) -> None:
        """Publish all outputs derived from one processed RGB frame."""
        self._publish_detections(header, faces)

        if faces:
            # Only the selected face is projected to 3D and converted to pose/TF.
            selected = self._filter_face_landmarks(faces[0])
            self._publish_selected_2d(selected)
            projected = self._project_selected_to_3d(selected)
            if projected is not None:
                self._publish_selected_3d(projected.points)
                self._publish_face_pose(header, projected)
        elif self.publish_empty_2d:
            self._reset_temporal_filter()
            self.landmarks_2d_pub.publish(Float32MultiArray())
        else:
            self._reset_temporal_filter()

        if self.publish_debug_image:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)

        self._publish_status("ok", f"faces={len(faces)}")

    def _publish_selected_2d(self, face: FaceLandmarkRecord) -> None:
        """Publish the selected face landmarks as a compact Float32MultiArray."""
        mouth = face.mouth_center
        msg = Float32MultiArray()
        msg.data = [
            float(face.left_eye[0]),
            float(face.left_eye[1]),
            float(face.right_eye[0]),
            float(face.right_eye[1]),
            float(face.nose[0]),
            float(face.nose[1]),
            float(mouth[0]),
            float(mouth[1]),
            float(face.confidence),
            float(face.center_x),
            float(face.center_y),
            float(face.width),
            float(face.height),
        ]
        self.landmarks_2d_pub.publish(msg)

    def _project_selected_to_3d(self, face: FaceLandmarkRecord) -> Optional[ProjectedFaceLandmarks]:
        """Convert selected 2D landmarks into 3D points using aligned depth."""
        camera_info_msg = self.camera_info_msg
        depth_msg = self.depth_msg
        depth_image = self.depth_image
        if not self.enable_depth_projection or camera_info_msg is None or depth_msg is None or depth_image is None:
            return None

        points_2d = [face.left_eye, face.right_eye, face.nose, face.mouth_center]
        points_3d = []
        for u, v in points_2d:
            # Median depth around the pixel is more robust than a single noisy sample.
            depth_m = self._sample_depth_m(depth_image, depth_msg.encoding, int(round(u)), int(round(v)))
            if depth_m is None:
                return None
            points_3d.append(self._deproject_pixel(camera_info_msg, float(u), float(v), depth_m))

        points_3d = self._filter_3d_points(points_3d)

        header = Header()
        header.stamp = depth_msg.header.stamp
        header.frame_id = depth_msg.header.frame_id or camera_info_msg.header.frame_id
        return ProjectedFaceLandmarks(header=header, points=points_3d)

    def _sample_depth_m(self, depth: np.ndarray, encoding: str, u: int, v: int) -> Optional[float]:
        """Sample a local depth window and return a filtered depth value in meters."""
        if depth is None or v < 0 or u < 0 or v >= depth.shape[0] or u >= depth.shape[1]:
            return None

        radius = max(0, self.depth_window_radius)
        y1 = max(0, v - radius)
        y2 = min(depth.shape[0], v + radius + 1)
        x1 = max(0, u - radius)
        x2 = min(depth.shape[1], u + radius + 1)
        patch = depth[y1:y2, x1:x2].astype(np.float32)

        if encoding in ("16UC1", "mono16"):
            # Orbbec aligned depth is commonly millimeters for 16-bit images.
            patch = patch / 1000.0
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _deproject_pixel(self, camera_info_msg: CameraInfo, u: float, v: float, z: float) -> Point:
        """Use pinhole intrinsics to convert one image pixel and depth into XYZ."""
        k = camera_info_msg.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]
        point = Point()
        point.x = (u - cx) * z / fx
        point.y = (v - cy) * z / fy
        point.z = z
        return point

    def _publish_selected_3d(self, points: List[Point]) -> None:
        """Publish selected 3D landmarks as flat xyz triplets."""
        msg = Float32MultiArray()
        for point in points:
            msg.data.extend([float(point.x), float(point.y), float(point.z)])
        self.landmarks_3d_pub.publish(msg)

    def _publish_face_pose(self, image_header: Header, projected: ProjectedFaceLandmarks) -> None:
        """Publish nose-centered face pose and optional TF in the target frame."""
        source_frame = projected.header.frame_id
        target_frame = self.target_frame or source_frame
        if not source_frame:
            self._publish_status("error", "cannot publish face pose: missing 3D landmark frame")
            return

        transform = self._lookup_transform(target_frame, source_frame)
        if transform is None:
            return

        rotation, translation = transform
        points = [
            self._transform_point_to_target(point, rotation, translation)
            for point in projected.points
        ]
        if len(points) < 3:
            return

        # Use the camera optical +Z, transformed into target_frame, to resolve the
        # face-plane normal direction consistently across frames.
        preferred_back_axis = rotation @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
        pose_rotation = self._compute_face_rotation(points[0], points[1], points[2], preferred_back_axis)
        if pose_rotation is None:
            return

        quat_x, quat_y, quat_z, quat_w = _rotation_matrix_to_quat(pose_rotation)
        nose = points[2]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = image_header.stamp
        pose_msg.header.frame_id = target_frame
        pose_msg.pose.position.x = float(nose[0])
        pose_msg.pose.position.y = float(nose[1])
        pose_msg.pose.position.z = float(nose[2])
        pose_msg.pose.orientation.x = quat_x
        pose_msg.pose.orientation.y = quat_y
        pose_msg.pose.orientation.z = quat_z
        pose_msg.pose.orientation.w = quat_w
        self.face_pose_pub.publish(pose_msg)

        if self.publish_face_tf:
            tf_msg = TransformStamped()
            tf_msg.header = pose_msg.header
            tf_msg.child_frame_id = self.face_frame_id
            tf_msg.transform.translation.x = pose_msg.pose.position.x
            tf_msg.transform.translation.y = pose_msg.pose.position.y
            tf_msg.transform.translation.z = pose_msg.pose.position.z
            tf_msg.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    def _lookup_transform(self, target_frame: str, source_frame: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Fetch source-to-target TF as a rotation matrix and translation vector."""
        if target_frame == source_frame:
            return np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)

        try:
            tf_msg = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except Exception as exc:
            self._publish_status(
                "error",
                f"tf lookup failed: target={target_frame}, source={source_frame}, error={exc}",
            )
            return None

        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        rotation = _quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z], dtype=np.float64)
        return rotation, translation

    def _transform_point_to_target(
        self,
        point: Point,
        rotation: np.ndarray,
        translation: np.ndarray,
    ) -> np.ndarray:
        """Apply a precomputed TF transform to one geometry_msgs Point."""
        source_point = np.array([point.x, point.y, point.z], dtype=np.float64)
        return rotation @ source_point + translation

    def _compute_face_rotation(
        self,
        left_eye: np.ndarray,
        right_eye: np.ndarray,
        nose: np.ndarray,
        preferred_back_axis: np.ndarray,
    ) -> Optional[np.ndarray]:
        """Estimate face orientation from eyes and nose in the target coordinate frame."""
        x_axis = _normalize(left_eye - right_eye, self.min_eye_distance_m)
        if x_axis is None:
            self._publish_status("error", "cannot publish face pose: eye distance is too small")
            return None

        # The eyes define the lateral axis; the nose-to-eye-center vector gives
        # a face plane. Their cross product estimates the face normal.
        nose_to_eye_center = 0.5 * (left_eye + right_eye) - nose
        z_axis = _normalize(np.cross(x_axis, nose_to_eye_center), self.min_plane_normal_norm)
        if z_axis is None:
            self._publish_status("error", "cannot publish face pose: face plane is degenerate")
            return None

        preferred_back_axis = _normalize(preferred_back_axis)
        if preferred_back_axis is not None and float(np.dot(z_axis, preferred_back_axis)) < 0.0:
            z_axis = -z_axis

        y_axis = _normalize(np.cross(z_axis, x_axis))
        if y_axis is None:
            self._publish_status("error", "cannot publish face pose: invalid y axis")
            return None

        z_axis = _normalize(np.cross(x_axis, y_axis))
        if z_axis is None:
            self._publish_status("error", "cannot publish face pose: invalid z axis")
            return None

        return np.column_stack((x_axis, y_axis, z_axis))

    def _publish_detections(self, header: Header, faces: List[FaceLandmarkRecord]) -> None:
        """Publish vision_msgs detections for all faces after NMS."""
        msg = Detection2DArray()
        msg.header = header
        for face in faces:
            detection_msg = Detection2D()
            detection_msg.header = header
            bbox = BoundingBox2D()
            bbox.center.position.x = float(face.center_x)
            bbox.center.position.y = float(face.center_y)
            bbox.center.theta = 0.0
            bbox.size_x = float(face.width)
            bbox.size_y = float(face.height)
            detection_msg.bbox = bbox

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "face"
            hypothesis.hypothesis.score = float(face.confidence)
            detection_msg.results.append(hypothesis)
            msg.detections.append(detection_msg)

        if faces or self.publish_empty_detections:
            self.detections_pub.publish(msg)

    def _publish_status(self, level: str, message: str) -> None:
        """Publish lightweight status text, throttling repeated OK messages."""
        now = time.time()
        if level == "ok" and now - self._last_status_time < 1.0:
            return
        self._last_status_time = now
        status = String()
        status.data = f"level={level}; message={message}; model={self.model_path}"
        self.status_pub.publish(status)


def main(args=None):
    """Start the ROS node and shut it down cleanly."""
    rclpy.init(args=args)
    node = None
    try:
        node = OrbbecFaceLandmarkNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
