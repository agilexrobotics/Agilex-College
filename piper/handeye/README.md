# Piper手眼标定

- 眼在手上：相机固定在机械臂末端，标定板不动，得到的是末端执行器到相机的变换矩阵 `T_ee_cam`, 抓取应用：`T_base_cam = T_base_ee × T_ee_cam`

- 眼在手外：标定板固定在机械臂末端，相机不动，得到的是基座坐标系到相机的变换矩阵 `T_base_cam`，定位应用：`T_ee_cam = T_ee_base × T_base_cam`

## 准备工作

- Ubuntu 22.04
- ROS2 Humble
- [在线生成标定板]( https://chev.me/arucogen/)（建议使用`Original ArUco`字典的标定板）
- [二维码识别程序](https://github.com/pal-robotics/aruco_ros/tree/humble-devel)
- [piper机械臂程序](https://github.com/agilexrobotics/piper_ros/tree/humble)
- [手眼标定程序](https://github.com/agilexrobotics/handeye_calibration_ros)
- [USB相机内参标定](https://github.com/kehuanjack/camera)（可选）

## 标定步骤

1. 创建工作空间，克隆功能包

    ```bash
    mkdir -p ~/handeye/src
    cd ~/handeye/src
    git clone -b humble-devel https://github.com/pal-robotics/aruco_ros.git
    git clone -b humble https://github.com/agilexrobotics/piper_ros.git
    git clone -b humble https://github.com/agilexrobotics/handeye_calibration_ros.git
    ```

2. 编译工作空间

    ```bash
    cd ~/handeye
    colcon build
    ```

3. 运行二维码识别程序

    ```bash
    source ~/handeye/src/install/setup.sh

    # 这里的 marker_id 和 marker_size 需要和生成的标定板实际信息一致
    ros2 launch aruco_ros single.launch.py eye:=left marker_id:=582 marker_size:=0.0677
    ```

4. 连接相机，运行相机程序，发布相机图像话题和相机信息话题，这里以 [IntelRealSense](https://github.com/IntelRealSense/realsense-ros) 的相机为例

    ```bash
    # 这里的 left 对应步骤3的 eye 参数
    # 需要将原相机图像话题和相机信息话题重映射为二维码识别程序所订阅的话题
    ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -p rgb_camera.color_profile:=640x480x60 \
    --remap /camera/camera/color/image_raw:=/stereo/left/image_rect_color \
    --remap /camera/camera/color/camera_info:=/stereo/left/camera_info
    ```

5. 连接piper机械臂，激活CAN模块，运行机械臂程序

    ```bash
    bash ~/handeye/src/piper_ros/can_activate.sh
    source ~/handeye/src/install/setup.sh
    ros2 launch piper start_single_piper.launch.py can_port:=can0
    ```

6. 运行piper手眼标定程序

    ```bash
    # mode 参数为 eye_in_hand 或 eye_to_hand，分别对应眼在手上和眼在手外的标定模式
    ros2 run handeye_calibration_ros handeye_calibration --ros-args \
    -p piper_topic:=/end_pose \
    -p marker_topic:=/aruco_single/pose \
    -p mode:=eye_in_hand
    ```

7. 控制机械臂，按照标定程序的终端提示进行标定，这里提供两种控制机械臂的方式：

    - 示教模式：点击示教按钮，直接拖动机械臂，该模式为mit模式，精度比位置速度模式低
    - [手柄控制](https://github.com/kehuanjack/Gamepad_PiPER)：可以使用位置速度模式控制机械臂，精度更高，需要配合手柄遥操程序使用

## 其他

1. 查看图像

    ```bash
    ros2 run image_view image_view --ros-args --remap /image:=/aruco_single/result
    ```

2. 查看二维码位姿

    ```bash
    ros2 topic echo /aruco_single/pose
    ```

3. 查看机械臂位姿

    ```bash
    ros2 topic echo /end_pose
    ```

4. 相机信息话题中包含相机的内参和畸变参数，二维码识别程序中需要使用这些信息

5. 其他相机，如工业相机、普通相机等，可能需要额外标定相机的内参和畸变参数，并将其封装成相机信息话题
