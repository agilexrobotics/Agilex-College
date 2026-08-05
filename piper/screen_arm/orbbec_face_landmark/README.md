# orbbec_face_landmark

> 本文件保留原项目的详细实验记录，其中 `/home/grubaxu` 路径仅是历史示例。
> 统一仓库的可移植安装与模型配置以 `screen_arm_ws/README.md` 为准。

`orbbec_face_landmark` 是放在 `src/advx` 下的独立 ROS 2 功能包，用于订阅奥比中光 OrbbecSDK_ROS2 发布的彩色图像，并输出人脸 2D 关键点；如果注册深度图、相机内参和 TF 可用，也会输出关键点 3D 位置以及人脸在相机基坐标系下的位姿。

当前节点已经在本机验证通过：

- 彩色图像输入正常。
- YOLOv5-Face ONNX 模型可以加载。
- `/face_landmarks_2d` 可以稳定发布人脸关键点。
- `/face_pose` 可以发布以鼻子为原点的人脸位姿。
- `/face_landmarks_debug_image` 可以发布调试图像。
- 默认使用 ONNX Runtime CUDA 推理；CUDA 不可用时自动回退到 OpenCV CPU。

## 功能

本节点完成以下工作：

- 订阅 Orbbec 彩色图像 `/camera/color/image_raw`。
- 使用 YOLOv5-Face ONNX 模型检测人脸框和 5 个人脸关键点。
- 从 5 个关键点中发布左眼、右眼、鼻尖、嘴巴中心。
- 可选订阅深度图 `/camera/depth/image_raw` 和彩色相机内参 `/camera/color/camera_info`，把关键点从 2D 像素反投影成 3D 点。
- 使用左眼、右眼、鼻子三个 3D 点构造人脸坐标系，并发布 `geometry_msgs/msg/PoseStamped`。
- 可选发布 `target_frame -> face` 的 TF。
- 对选中人脸的 2D 关键点、深度反投影后的 3D 点和最终位姿进行时间 EMA 滤波，降低检测和深度噪声造成的抖动。
- 使用 ONNX Runtime CUDA 加速 YOLOv5-Face 推理，减少 CPU 推理造成的帧率瓶颈。
- 发布调试图像，方便用 `rqt_image_view` 检查关键点是否落在人脸上。

## 输入话题

根据当前相机运行时的 `ros2 topic list`，默认输入话题配置为：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
```

其中：

- `/camera/color/image_raw`：彩色图像，用于 YOLOv5-Face 人脸关键点检测。
- `/camera/color/camera_info`：彩色相机内参，用于把 2D 像素点反投影到 3D。
- `/camera/depth/image_raw`：深度图，用于获取关键点深度。

如果要让深度图和彩色图对齐，启动 OrbbecSDK_ROS2 时需要开启 `depth_registration:=true`，例如：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true
```

你的话题列表里有 `/camera/depth_to_color`，但它不是 `sensor_msgs/msg/Image` 图像话题，不能作为本节点的深度输入。不要把它写成 `aligned_depth_topic`。

## 输出话题

本节点默认发布：

- `/face_landmarks_2d`：`std_msgs/msg/Float32MultiArray`，选中人脸的 2D 关键点。
- `/face_landmarks_3d`：`std_msgs/msg/Float32MultiArray`，选中人脸的 3D 关键点；只有深度图和内参都可用时发布。
- `/face_pose`：`geometry_msgs/msg/PoseStamped`，选中人脸在 `target_frame` 下的位姿，默认 `target_frame=camera_link`。
- `/face_detections_2d`：`vision_msgs/msg/Detection2DArray`，所有人脸检测框。
- `/face_landmarks_debug_image`：`sensor_msgs/msg/Image`，画好人脸框和关键点的调试图。
- `/orbbec_face_landmark/status`：`std_msgs/msg/String`，节点状态和错误信息。

默认还会发布 TF：

```text
camera_link -> face
```

当前 Orbbec launch 发布的相机基坐标是 `camera_link`。如果你的整机 TF 树里有真正的 `base_link -> camera_link` 外参，可以把配置里的 `target_frame` 改成 `base_link`。

## 2D 关键点格式

`/face_landmarks_2d` 的数组格式固定为：

```text
[
  left_eye_x, left_eye_y,
  right_eye_x, right_eye_y,
  nose_x, nose_y,
  mouth_center_x, mouth_center_y,
  confidence,
  bbox_center_x, bbox_center_y,
  bbox_width, bbox_height
]
```

坐标单位是原始彩色图像上的像素。

## 3D 关键点格式

`/face_landmarks_3d` 的数组格式固定为：

```text
[
  left_eye_x, left_eye_y, left_eye_z,
  right_eye_x, right_eye_y, right_eye_z,
  nose_x, nose_y, nose_z,
  mouth_center_x, mouth_center_y, mouth_center_z
]
```

3D 坐标单位是米，坐标系遵循彩色相机 `CameraInfo` 对应的 optical frame 约定。深度值不是取单像素，而是在关键点附近窗口取中位数，减少深度噪声影响。

注意：如果 `/camera/depth/image_raw` 的 `frame_id` 仍然是 `camera_depth_optical_frame`，并且尺寸与彩色图不同，说明深度尚未对齐到彩色图。此时 `/face_landmarks_3d` 只能用于链路测试，不能作为准确 3D 坐标使用。需要重新启动 Orbbec 驱动并启用 `depth_registration:=true`。

## 人脸位姿定义

`/face_pose` 和 `face` TF 的坐标定义为：

```text
origin: 鼻子 3D 点
x axis: 从右眼指向左眼，平行左右眼连线
xy plane: 左眼、右眼、鼻子三个点构成的平面
z axis: 三点平面法向，方向翻转到指向脑后
```

节点会先把左眼、右眼、鼻子的 3D 点从 `camera_color_optical_frame` 转到 `target_frame`，再计算人脸姿态。`z` 轴的正方向默认按彩色光学坐标系的 `+Z` 方向选择，也就是远离相机方向；人在相机前方看向相机时，这就是脑后方向。

## 目录结构

```text
orbbec_face_landmark/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── orbbec_face_landmark.yaml
├── launch/
│   └── orbbec_face_landmark.launch.py
├── scripts/
│   └── orbbec_face_landmark_node
└── orbbec_face_landmark/
    ├── __init__.py
    └── orbbec_face_landmark_node.py
```

## 本机环境

当前本机默认环境如下：

```text
ROS 2: Humble
工作空间: /home/grubaxu/ros2_ws
功能包路径: /home/grubaxu/ros2_ws/src/advx/orbbec_face_landmark
Orbbec 驱动路径: /home/grubaxu/ros2_ws/src/advx/OrbbecSDK_ROS2
推理虚拟环境: /home/grubaxu/yolov5/.venv
模型路径: /home/grubaxu/yolov5-face/weights/yolov5n_face.onnx
```

节点通过 `scripts/orbbec_face_landmark_node` 显式使用：

```text
/home/grubaxu/yolov5/.venv/bin/python
```

这样模型推理依赖不会污染系统 Python 环境。

## 依赖

系统 ROS 依赖：

```text
rclpy
sensor_msgs
std_msgs
geometry_msgs
vision_msgs
cv_bridge
launch
launch_ros
ament_cmake
ament_cmake_python
```

Python 推理依赖：

```text
opencv-python
numpy
```

当前 ONNX 推理使用 OpenCV DNN，不需要 `onnxruntime`。

如果缺少 ROS 包，可以安装：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-image-transport \
  ros-humble-rqt-image-view
```

## 模型文件

默认模型路径：

```text
/home/grubaxu/yolov5-face/weights/yolov5n_face.onnx
```

下载命令：

```bash
mkdir -p /home/grubaxu/yolov5-face/weights
curl -L -o /home/grubaxu/yolov5-face/weights/yolov5n_face.onnx \
  https://github.com/yakhyo/yolov5-face-onnx-inference/releases/download/weights/yolov5n_face.onnx
```

本机已经下载完成。移植到其他电脑时，需要确认该文件存在：

```bash
ls -lh /home/grubaxu/yolov5-face/weights/yolov5n_face.onnx
```

如果其他电脑用户名不是 `grubaxu`，建议修改 `config/orbbec_face_landmark.yaml` 里的 `model_path`，或者保持相同目录结构。

## 构建

```bash
cd /home/grubaxu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orbbec_face_landmark
source install/setup.bash
```

确认 ROS 2 能找到节点：

```bash
ros2 pkg executables orbbec_face_landmark
```

正常输出应包含：

```text
orbbec_face_landmark orbbec_face_landmark_node
```

## 启动顺序

先启动 Orbbec 相机驱动。以 `dabai.launch.py` 为例：

```bash
source /opt/ros/humble/setup.bash
source /home/grubaxu/ros2_ws/install/setup.bash
ros2 launch orbbec_camera dabai.launch.py depth_registration:=true
```

确认相机话题存在：

```bash
ros2 topic list
```

至少应看到：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
```

再启动人脸关键点节点：

```bash
source /opt/ros/humble/setup.bash
source /home/grubaxu/ros2_ws/install/setup.bash
ros2 launch orbbec_face_landmark orbbec_face_landmark.launch.py
```

## 验证

查看节点状态：

```bash
ros2 topic echo /orbbec_face_landmark/status
```

正常检测到人脸时，类似：

```text
data: level=ok; message=faces=1; model=/home/grubaxu/yolov5-face/weights/yolov5n_face.onnx
```

查看 2D 关键点：

```bash
ros2 topic echo /face_landmarks_2d
```

查看人脸框：

```bash
ros2 topic echo /face_detections_2d
```

查看 3D 关键点：

```bash
ros2 topic echo /face_landmarks_3d
```

查看人脸位姿：

```bash
ros2 topic echo /face_pose
```

查看 TF：

```bash
ros2 run tf2_ros tf2_echo camera_link face
```

查看调试图像：

```bash
rqt_image_view /face_landmarks_debug_image
```

查看发布频率：

```bash
ros2 topic hz /face_landmarks_2d
```

本机 RTX 3060 使用 ONNX Runtime CUDA 推理时，模型推理约 `13-15 ms/帧`，
完整的预处理、推理和后处理约 `94 FPS`。实际 ROS 输出帧率受相机发布帧率限制，
当前相机为约 `30 Hz`。

## 配置项

配置文件：

```text
/home/grubaxu/ros2_ws/src/advx/orbbec_face_landmark/config/orbbec_face_landmark.yaml
```

常用配置：

```yaml
color_image_topic: /camera/color/image_raw
color_camera_info_topic: /camera/color/camera_info
aligned_depth_topic: /camera/depth/image_raw
landmarks_3d_topic: /face_landmarks_3d
face_pose_topic: /face_pose
target_frame: camera_link
face_frame_id: face
publish_face_tf: true
inference_backend: onnxruntime
device: cuda:0
model_path: /home/grubaxu/yolov5-face/weights/yolov5n_face.onnx
confidence_threshold: 0.6
max_fps: 0.0
enable_depth_projection: true
enable_temporal_filter: true
landmark_filter_alpha: 0.35
depth_filter_alpha: 0.25
```

其中 `alpha` 越小越平滑但响应越慢，`alpha=1.0` 等价于关闭对应级别的滤波。默认 2D 关键点使用 `0.35`，3D 点和人脸位姿使用 `0.25`。没有检测到人脸时会清空滤波状态，下一次检测直接重新初始化。

`max_fps` 设置为 `0` 或负数时不启用软件限频，推理线程会持续处理最新的相机帧。
GPU 推理速度高于当前相机帧率时，建议保持为 `0.0`，由相机帧率决定输出频率。

默认配置使用 ONNX Runtime CUDA 推理。如果 CUDA provider 不可用，节点会自动回退到 OpenCV CPU。需要强制使用 CPU 时，可以配置：

```yaml
inference_backend: opencv
device: cpu
```

GPU 环境安装依赖：

```bash
source /home/grubaxu/yolov5/.venv/bin/activate
pip install onnxruntime-gpu
pip install \
  nvidia-cuda-runtime-cu12 \
  nvidia-cuda-nvrtc-cu12 \
  nvidia-cuda-cupti-cu12 \
  nvidia-cublas-cu12 \
  nvidia-cudnn-cu12 \
  nvidia-cufft-cu12 \
  nvidia-curand-cu12 \
  nvidia-cusolver-cu12 \
  nvidia-cusparse-cu12 \
  nvidia-nvjitlink-cu12
```

## 移植到其他电脑

### 1. 准备 ROS 2 工作空间

假设新电脑也使用 ROS 2 Humble：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

把以下源码放到新电脑的 `src/advx` 下：

```text
advx/OrbbecSDK_ROS2
advx/orbbec_face_landmark
```

如果机械臂相关代码也需要一起运行，再同步对应机械臂功能包。

### 2. 安装 Orbbec 驱动依赖

按照 `advx/OrbbecSDK_ROS2` 自带 README 安装 Orbbec SDK、udev 规则和 ROS 依赖。

通常需要至少执行：

```bash
cd ~/ros2_ws/src/advx/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
```

udev 规则安装后，重新插拔相机。

### 3. 创建推理虚拟环境

推荐保持和本机一致的路径：

```bash
python3 -m venv /home/$USER/yolov5/.venv
source /home/$USER/yolov5/.venv/bin/activate
pip install --upgrade pip
pip install opencv-python numpy
```

如果新电脑路径不是 `/home/grubaxu/yolov5/.venv/bin/python`，需要修改：

```text
orbbec_face_landmark/scripts/orbbec_face_landmark_node
```

把：

```bash
VENV_PYTHON="/home/grubaxu/yolov5/.venv/bin/python"
```

改成新电脑上的 venv Python 路径，例如：

```bash
VENV_PYTHON="/home/new_user/yolov5/.venv/bin/python"
```

### 4. 下载模型

推荐保持默认目录结构：

```bash
mkdir -p /home/$USER/yolov5-face/weights
curl -L -o /home/$USER/yolov5-face/weights/yolov5n_face.onnx \
  https://github.com/yakhyo/yolov5-face-onnx-inference/releases/download/weights/yolov5n_face.onnx
```

如果新电脑用户名不是 `grubaxu`，还需要修改：

```text
orbbec_face_landmark/config/orbbec_face_landmark.yaml
```

把：

```yaml
model_path: /home/grubaxu/yolov5-face/weights/yolov5n_face.onnx
```

改成：

```yaml
model_path: /home/new_user/yolov5-face/weights/yolov5n_face.onnx
```

### 5. 构建功能包

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orbbec_face_landmark
source install/setup.bash
```

### 6. 修改话题配置

在新电脑上启动相机后查看话题：

```bash
ros2 topic list
```

如果相机命名空间不是 `/camera`，修改：

```text
orbbec_face_landmark/config/orbbec_face_landmark.yaml
```

例如相机发布的是 `/my_camera/color/image_raw`，则改成：

```yaml
color_image_topic: /my_camera/color/image_raw
color_camera_info_topic: /my_camera/color/camera_info
aligned_depth_topic: /my_camera/depth/image_raw
```

### 7. 运行验证

```bash
ros2 launch orbbec_face_landmark orbbec_face_landmark.launch.py
ros2 topic echo /face_landmarks_2d
rqt_image_view /face_landmarks_debug_image
```

## 常见问题

### 1. 启动时报找不到模型文件

检查：

```bash
ls -lh /home/grubaxu/yolov5-face/weights/yolov5n_face.onnx
```

如果文件不在默认路径，修改 `model_path`。

### 2. 没有 `/face_landmarks_2d` 输出

先看状态：

```bash
ros2 topic echo /orbbec_face_landmark/status
```

再确认彩色图像有数据：

```bash
ros2 topic echo /camera/color/image_raw --once
```

如果画面里没有人脸，默认不会发布 `/face_landmarks_2d`。可以打开调试图像确认画面内容：

```bash
rqt_image_view /face_landmarks_debug_image
```

### 3. 有 2D 输出，但 3D 输出不准确

检查深度图是否和彩色图对齐：

```bash
ros2 topic echo /camera/depth/image_raw --once
ros2 topic echo /camera/color/image_raw --once
```

如果尺寸不同，或者 depth 的 `frame_id` 仍然是 `camera_depth_optical_frame`，需要使用 `depth_registration:=true` 重启 Orbbec 驱动。

### 4. ONNX Runtime CUDA 报错

默认配置使用 ONNX Runtime CUDA。如果启动日志中没有
`CUDAExecutionProvider`，或者 CUDA provider 初始化失败，可以临时回退到 CPU：

```yaml
device: cpu
```

### 5. 推理速度慢

先确认启动日志包含 `inference_backend=onnxruntime` 和
`inference_device=cuda`，再检查 GPU 是否有负载：

```bash
nvidia-smi
ros2 topic hz /face_landmarks_2d
ros2 topic hz /face_pose
```

当前节点使用“只保留最新图像”的推理队列，避免处理旧帧造成延迟。
`max_fps: 0.0` 表示不额外限速；输出帧率通常接近相机的彩色图像帧率。

## 交接检查清单

交接给其他同事时，至少确认以下内容：

- Orbbec 相机能正常发布 `/camera/color/image_raw`。
- `/camera/color/camera_info` 能正常发布，并且 `frame_id` 是彩色相机 optical frame。
- 模型文件 `yolov5n_face.onnx` 存在。
- `scripts/orbbec_face_landmark_node` 里的 `VENV_PYTHON` 路径在目标电脑上真实存在。
- `config/orbbec_face_landmark.yaml` 里的 `model_path` 和相机话题名与目标电脑一致。
- `colcon build --packages-select orbbec_face_landmark` 能通过。
- `ros2 topic echo /face_landmarks_2d` 能看到关键点数组。
- `rqt_image_view /face_landmarks_debug_image` 能看到画好框和关键点的图像。
