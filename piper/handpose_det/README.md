## <div align="center">文档</div>      
此仓库通过深度相机实现低成本简单遥操作功能（仅实现机械臂末端的位置控制，无姿态控制）。

## <div align="center">硬件设备</div>
- [奥比中光 Petrel](https://orbbec.com.cn/index/Product/info.html?cate=38&id=28) （带对齐的深度图像与RGB图像：640*400@30fps）
- （可选）：intel realsense D435（带对齐的深度图像与RGB图像：640*480@30fps）
- NVIDIA 3050laptop （带3050移动端的笔记本）
- Agilex robotics Piper机械臂

## <div align="center">环境</div>
<details open>
<summary>软件环境</summary>

#### 桌面
- ubuntu20.04
- ROS noetic

</details>

## <div align="center">环境部署</div>

#### 安装项目依赖的ROS包

````bash
sudo apt install -y ros-noetic-sensor-msgs ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-msgs ros-noetic-image-geometry ros-noetic-pcl-conversions ros-noetic-pcl-ros ros-noetic-message-filters
````

#### 克隆并编译项目

7. 克隆并编译此功能包
````bash
cd your_ws/src
git cloen https://github.com/agilexrobotics/Agilex-College.git
cd ..
catkin_make
source devel/setup.bash
````

## <div align="center">运行</div>

#### 启动相机节点

- 奥比中光Petrel

需要相机能<font color=red>自动对齐</font>RGB图像与深度图像，此项目暂时没有开发自动对齐功能，对齐后的深度图像与RGB图像长宽一致。此项目使用[奥比中光 Petrel](https://orbbec.com.cn/index/Product/info.html?cate=38&id=28)深度相机作为功能测试，具体配置运行示例以及设置请参考[奥比中光相机驱动仓库](https://github.com/orbbec/ros_astra_camera.git)

````bash
source devel/setup.bash
roslaunch roslaunch astra_camera dabai_dc1.launch 
````

- Intel realsense D435

D435深度相机有自动对齐RGB图像与深度图像的功能，根据realsense官方部署文档安装深度相机驱动和ROS功能包即可

````bash
roslaunch realsense2_camera rs_aligned_depth.launch
````

#### 定义Piper的home点

- Piper驱动内部的home点定义在Piper使能后，可能无法对Piper的X、Y轴坐标进行控制，需要手动定义Piper的home点。项目中定义Piper的Home点为
````python
self.endPosX = -0.344 # Home点的X坐标
self.endPosY = 0.0 # Home点的Y坐标
self.endPosZ = 0.110 # Home点的Z坐标
````

- 将Piper移动到Home点

````bash
rostopic pub /pos_cmd piper_msgs/PosCmd "{
x: -0.344,
y: 0.0,
z: 0.110,
roll: 0.0,
pitch: 0.0,
yaw: 0.0,
gripper: 0.0,
mode1: 1,
mode2: 0
}"
````

- 自定义Home点流程
- 1. 启动`piper_ros`中的piper start_single_piper.launch

````bash
roslaunch piper start_single_piper.launch 
````
- 2. 启动`piper_ros`中的`piper_pinocchio.py `运动学逆解算工具皮诺曹

````bash
python piper_pinocchio/piper_pinocchio.py 
````

- 3. 启动`handpose_det/scripts`里的Piper X-Y-Z三轴单独控制节点`test_arm.py`， 控制机械臂到期望的位置

- 4. 查看`piper_ros`节点的`/end_pose`话题，可以确定Piper的末端位姿

````bash
rostopic echo /end_pose
````

- 5. 修改`handpose_det.py`中Home点的值

#### 启动手势识别节点

````bash
source devel/setup.bash
roslaunch handpose_det handpose_det.py
````

- 手势识别节点的用法为：首先将手握拳，此时节点识别到手势为`FIST`,持续三秒钟，节点会将手势变换前一刻的3D位置记作一个`基坐标系`的原点，手势非`FIST`时会实时计算当前手势的3D位置与`基坐标系`的原点的差值，将差值加上Home点的值，即为机械臂末端的期望位置，然后节点会将期望位置发送给`piper_ros`中的`piper_pinocchio.py `运动学逆解算工具皮诺曹，实现机械臂末端位置的控制

