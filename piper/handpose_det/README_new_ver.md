# 此项目通过深度相机实现低成本简单遥操作功能

## 硬件设备

- 奥比中光 Petrel（带对齐的深度图像与RGB图像：640*400@30fps）
- （可选）：intel realsense D435（带对齐的深度图像与RGB图像：640*480@30fps）
- （可选）：奥比中光 gemini （带对齐的深度图像与RGB图像：640*400@60fps）
- NVIDIA 3050laptop （带3050移动端的笔记本）
- AgileX robotics PiPer机械臂

## 软件环境

- Ubuntu 20.04
- ROS noetic

## 依赖

- 安装依赖的ROS功能包

````bash
sudo apt install -y ros-noetic-sensor-msgs ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-msgs ros-noetic-image-geometry ros-noetic-pcl-conversions ros-noetic-pcl-ros ros-noetic-message-filters

git cloen https://github.com/agilexrobotics/Agilex-College.git
cd Agilex-College/piper/
cp -r piper_kinematics/ your_ws/src/
catkin_make
````

- PIPER机械臂驱动部署请参考：https://github.com/agilexrobotics/piper_sdk/blob/1_0_0_beta/README(ZH).MD

- PIPER机械臂ROS控制节点部署参考：https://github.com/agilexrobotics/piper_ros/blob/noetic/README.MD

- 克隆并编译此功能包

````bash
cd your_ws/src
git cloen https://github.com/agilexrobotics/Agilex-College.git
cd Agilex-College/piper/
cp -r handpose_det/ your_ws/src/
cd your_ws/
catkin_make
source devel/setup.bash
````

## 启动相机ROS节点

- 需要相机能自动对齐深度图像与彩色图像（D2C），此项目暂时不支持没有自动对齐功能的相机，对齐后的深度图像与RGB图像长宽一致。此项目使用[奥比中光 Genimi](https://orbbec.com.cn/index/Product/info.html?cate=38&id=28)深度相机作为功能测试，具体配置运行示例以及设置请参考[奥比中光相机驱动仓库](https://github.com/orbbec/ros_astra_camera.git)

- 奥比中光 Genimi使用下面的命令启动，需要先在*gemini.launch*文件中配置深度对齐功能

````bash
source devel/setup.bash
roslaunch roslaunch astra_camera gemini.launch 
````

- realsense D435使用下面的命令启动

````bash
roslaunch realsense2_camera rs_aligned_depth.launch
````

## 修改机械臂urdf文件

- 将机械臂urdf文件中表示机械臂基座的arm_base坐标系名称改为base_link
- 参考handpose_det/models/modified_piper.urdf或者modified_piper_without_camera.urdf,如果直接替换[piper_ros](https://github.com/agilexrobotics/piper_ros/)中的模型，注意修改launch文件

- 将连接到机械臂的can模块插入PC，查找机械臂CAN总线,并启动机械臂驱动节点

````bash
# 寻找机械臂的CAN端口
./find_all_can_port.sh
# 连接到机械臂的CAN端口
./can_activate.sh

roslaunch piper start_single_piper.launch
````

- 使用*piper_kinematics*中的交互式标记*interactive_pose_marker*工具，通过拖拽机械臂到达期望的home点

- 见[piper_kinematics](https://github.com/agilexrobotics/Agilex-College/tree/master/piper/piper_kinematics)中的*interactive_pose_marker*工具

- 启动*piper_kinematics*中的逆解节点，下达对Piper的控制指令到控制节点

````bash
rosrun piper_kinematics piper_ik_node 
rosrun piper_kinematics interactive_pose_marker.py
````

### 定义手部姿态的home点

- 启动手部姿态检测节点

````bash
rosrun handpose_det handpose_det_new_ik.py
````

- 启动rviz实时观察图像检测情况，其中MarkerArray中的红色圆点为指关节对应点，绿色圆点为手掌形心

- 保持握拳5秒姿势不动，五秒结束后会定义当前手部所处的姿态为手部坐标系的原点，随后张开五指开始映射手部相在手部坐标系的原点相对的姿态。（选取手部坐标系的原点时候，尽量选择图像中心的部位，且不宜距离相机过远导致手部识别不稳定，也不宜过近导致因相机视野受限导致的遥操作空间过小）

- 调整大拇指与食指的距离控制夹爪开合


# 完整运行流程

```bash
cd your_ws
source devel/setup.sh

# 1.激活CAN通信
cd piper_ros
./can_activate.sh

# 2.启动机械臂控制ROS节点
roslaunch piper start_single_piper_rviz.launch

# 3.启动相机驱动节点
roslaunch realsense2_camera rs_aligned_depth.launch

# 4.启动手势检测节点
# 注意修改 scripts/handpose_det_new_ik.py 中的话题名称以适配你使用的相机
roslaunch handpose_det hand_det_6d_pose.launch 

# 5.启动piper逆解节点
roslaunch piper_kinematics piper_ik.launch

# 6.拖动interactive_marker来定义机械臂末端初始点

# 7.如图所示相对位置关系摆放相机

# 8.右手握拳5秒以确定手部坐标系

# 9.张开手并拢四指以控制机械臂六个自由度、拇指控制夹爪开合
```

# 图示运行流程

1. 启动CAN通信
![alt text](doc/image.png)
2. 启动机械臂控制ROS节点
![alt text](doc/image-1.png)
3. 启动相机驱动节点
![alt text](doc/image-2.png)
4. 启动手势检测节点
![alt text](doc/image-3.png)
5. 启动piper逆解节点
![alt text](doc/image-4.png)
6. 拖动interactive_marker来定义机械臂末端初始点
![alt text](doc/image-5.png)
![alt text](doc/image-6.png)
![alt text](doc/image-7.png)
7. 如图所示相对位置关系摆放相机
![alt text](doc/image-8.png)
8. 右手握拳5秒以确定手部坐标系
![alt text](doc/image-9.png)
9. 张开手并拢四指以控制机械臂六个自由度、拇指控制夹爪开合