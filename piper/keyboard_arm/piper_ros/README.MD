# piper 机械臂ROS(humble)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![bash](https://img.shields.io/badge/bash-green.svg)

[EN](README(EN).MD)

python_version >= "3.10"

Test:

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

```shell
git clone https://github.com/agilexrobotics/piper_ros.git
```

```shell
cd piper_ros
git checkout humble
```

|说明 |文档|
|---|---|
|Moveit|[Moveit README](src/piper_moveit/README.md)|
|仿真|[仿真 README](src/piper_sim/README.md)|

## 0 注意URDF版本

S-V1.6-3固件版本以前的DH参数为各关节接触限位处为初始坐标得到

S-V1.6-3固件版本以后的DH参数将j2和j3的坐标系偏移了2度

现默认URDF为后者

|firmware version |urdf|
|---|---|
|current version < S-V1.6-3|`piper_description_old.urdf`|
|current version >= S-V1.6-3|`piper_description.urdf`|

## 1 安装方法

### 1.1 安装依赖

注: python-can版本应高于4.3.1

```shell
pip3 install python-can
pip3 install scipy
```

```shell
pip3 install piper_sdk
```

```shell
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

## 2 使用can模块

注意此处的can模块仅支持机械臂自带的can模块，不支持其它can模块

安装can工具

```shell
sudo apt update && sudo apt install can-utils ethtool
```

这两个工具用于配置 CAN 模块

如果执行bash脚本出现`ip: command not found`，请安装ip指令，一般是`sudo apt-get install iproute2`

### 2.1 寻找can模块

执行

```bash
bash find_all_can_port.sh
```

输入密码后，如果can模块插入了电脑，并被电脑检测到，输出类似如下：

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
```

如果有多个，输出类似如下：

```bash
Both ethtool and can-utils are installed.
Interface can0 is connected to USB port 3-1.4:1.0
Interface can1 is connected to USB port 3-1.1:1.0
```

有多少个can模块就会有多少行类似`Interface can1 is connected to USB port 3-1.1:1.0`的输出

其中`can1`是系统找到的can模块名字，`3-1.1:1.0`是该can模块所链接的usb端口

如果之前已经激活过can模块并其名为其它名字，这里假设名字为`can_piper`则输出如下

```bash
Both ethtool and can-utils are installed.
Interface can_piper is connected to USB port 3-1.4:1.0
Interface can0 is connected to USB port 3-1.1:1.0
```

如果没有检测到can模块，则只会输出如下：

```bash
Both ethtool and can-utils are installed.
```

### 2.2 激活单个can模块, **此处使用`can_activate.sh`脚本**

激活单个can模块的情况分两种，一种是电脑只链接了一个can模块，一种是多个can模块插入电脑但是只激活其中一个。

#### 2.2.1 pc只插入一个usb转can模块

直接执行

```bash
bash can_activate.sh can0 1000000
```

此处`can0`可以改为任意名字，`1000000`为波特率，使用机械臂的情况下，不可更改

#### 2.2.2 pc插入多个usb转can模块， 但每次只激活一个can模块

注： 此处用于同时使用机械臂和底盘的的情况

(1) 查看can模块插在usb端口的硬件地址。拔掉所有can模块，只将连接到机械臂的can模块插入PC，执行

```shell
bash find_all_can_port.sh
```

并记录下`USB port`的数值，例如`3-1.4:1.0`

(2) 激活can设备。假设上面的`USB port`数值为`3-1.4:1.0`，执行：

```bash
bash can_activate.sh can_piper 1000000 "3-1.4:1.0"
```

解释：**3-1.4:1.0硬件编码的usb端口插入的can设备，名字被重命名为can_piper，设定波特率为1000000，并激活**

(3) 检查是否激活成功

执行`ifconfig`查看是否有`can_piper`，如果有则can模块设置成功

### 2.3 同时激活多个can模块，**此处使用`can_muti_activate.sh`脚本**

首先确定有多少个官方can模块被插入到电脑，这里假设是2

注：**若当前电脑插入了5个can模块，可以只激活指定的can模块**

#### 2.3.1 记录每个can模块对应的usb口硬件地址

逐个拔插can模块并一一记录每个模块对应的usb口硬件地址

在`can_muti_activate.sh`中，`USB_PORTS`参数中元素的数量为预激活的can模块数量，现假设为2

(1) 然后can模块中的其中一个单独插入PC，执行

```shell
bash find_all_can_port.sh
```

并记录下`USB port`的数值，例如`3-1.4:1.0`

(2) 接着插入下一个can模块，注意**不可以**与上次can模块插入的usb口相同，然后执行

```shell
bash find_all_can_port.sh
```

记录下第二个can模块的`USB port`的数值，例如`3-1.1:1.0`

注：**如果未曾激活过，则第一个插入的can模块会默认是can0，第二个为can1，若激活过，名字为之前激活过的名称**

#### 2.3.2 预定义USB 端口、目标接口名称及其比特率

假设上面的操作记录的`USB port`数值分别为`3-1.4:1.0`、`3-1.1:1.0`，则将下面的`USB_PORTS["1-9:1.0"]="can_left:1000000"`的中括号内部的双引号内部的参数换为`3-1.4:1.0`和`3-1.1:1.0`.

最终结果为：

```bash
USB_PORTS["3-1.4:1.0"]="can_left:1000000"
USB_PORTS["3-1.1:1.0"]="can_right:1000000"
```

解释：**3-1.4:1.0硬件编码的usb端口插入的can设备，名字被重命名为can_left，波特率为1000000，并激活**

#### 2.3.3 激活多个can模块

执行`bash can_muti_activate.sh`

#### 2.3.4 查看多个can模块是否设置成功

执行`ifconfig`查看是不是有`can_left`和`can_right`

## 3 运行节点

编译

```shell
cd piper_ros
colcon build
source install/setup.bash
```

### 3.1 单个机械臂启动控制

节点名`piper_single_ctrl`

param

```shell
can_port:要打开的can路由名字
auto_enable:是否自动使能，True则开启程序就自动使能
# 注意这个设置为False，中断程序后再启动节点，机械臂会保持上次启动程序的状态
# 若上次启动程序机械臂状态为使能，则中断程序后再启动节点，机械臂仍为使能
# 若上次启动程序机械臂状态为失能，则中断程序后再启动节点，机械臂仍为失能
gripper_exist:是否有末端夹爪，True则说明有末端夹爪，会开启夹爪控制
rviz_ctrl_flag:是否使用rviz来发送关节角消息，True则接收rviz发送的关节角消息
gripper_val_mutiple:设置夹爪控制倍数
# 由于rviz中的joint7范围是[0,0.04]，而真实夹爪行程为0.08m，打开rviz控制真实夹爪需要设置夹爪二倍
```

有两个启动单臂的launch文件`start_single_piper_rviz.launch.py` 和 `start_single_piper.launch.py`

前者是可以一起将rviz启动，然后可以拖动滑动条控制机械臂

启动控制节点，以下几种方式任选一种运行即可

ros2 run 方式

```shell
# 启动节点
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2
# 设置日志等级 --log-level DEBUG/INFO/WARN/ERROR
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2 --log-level WARN
```

`ros2 topic list`

```bash
/arm_status #机械臂状态，详见下文
/enable_flag #使能标志位，发送给节点，发送true用来使能
/end_pose #机械臂末端位姿状态反馈
/end_pose_stamped #带有时间戳的endpose消息
/joint_ctrl #机械臂有关控制消息的反馈
/joint_ctrl_single #订阅关节消息，给这个消息发送关节位置能控制机械臂运动
/joint_states_single #机械臂关节状态反馈
/joint_states_feedback #同上，只是修改了topic名字
/pos_cmd #末端控制消息， xyzrpy，夹爪单位为m，mode1和mode2参数默认0即可
```

ros2 launch 方式

launch remap了`/joint_ctrl_single`topic，令`/joint_states`为控制机械臂的消息，方便与rviz连通，令rviz能够控制实际机械臂

```shell
# 启动节点
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2
# 设置日志等级 --log-level DEBUG/INFO/WARN/ERROR
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2 --log-level WARN
# 也可以用launch节点
ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false gripper_exist:=false gripper_val_mutiple:=2
# 增加设定log_level
ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false gripper_exist:=false gripper_val_mutiple:=2 log_level:=warn
# 或，会以默认参数运行
ros2 launch piper start_single_piper.launch.py
# 也可以用rviz开启控制,需要更改的参数如上
ros2 launch piper start_single_piper_rviz.launch.py
```

`ros2 topic list`

```shell
/arm_status #机械臂状态，详见下文
/enable_flag #使能标志位，发送给节点，发送true用来使能
/end_pose #机械臂末端位姿状态反馈
/end_pose_stamped #带有时间戳的endpose消息
/joint_ctrl #机械臂有关控制消息的反馈
/joint_states #订阅关节消息，给这个消息发送关节位置能控制机械臂运动
/joint_states_single #机械臂关节状态反馈
/joint_states_feedback #同上，只是修改了topic名字
/pos_cmd #末端控制消息， xyzrpy，夹爪单位为m，mode1和mode2参数默认0即可
```

ros2 service list

```shell
/enable_srv #机械臂使能服务端
```

使能机械臂

```shell
# call 服务端
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ true\ 
# pub topic
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ true\ 
```

失能机械臂

```shell
# call 服务端
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ false\ 
# pub topic
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ false\ 
```

发布关节消息

注意，机械臂会抬起，请确保机械臂工作范围内无障碍

机械臂会以默认速度的百分之10运动，夹爪力矩设定0.5N

速度范围限制为1-100,小于1时视为1

位置如下

```shell
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}"

```

### 3.2 单个机械臂启动读取关节消息并在rviz中跟随显示

启动读取关节数据节点

```bash
# 有夹爪
ros2 run piper piper_read_slave_joint --ros-args -p can_port:=can0 -p gripper_exist:=true
```

```bash
# 无夹爪
ros2 run piper piper_read_slave_joint --ros-args -p can_port:=can0 -p gripper_exist:=false
```

执行后仅有一个`/joint_states`用作机械臂消息的反馈

其中有夹爪启动的话消息内容会有`joint7`,`joint8`的数据，这个是为了rviz作为跟随显示功能的时候所需要的

启动rviz显示跟随launch

```bash
# 有夹爪
ros2 launch piper_description  display_urdf_follow.launch.py
```

```bash
# 无夹爪
ros2 launch piper_description  display_no_gripper_urdf_follow.launch.py
```

note:**如果只启动rviz显示，不启动关节数据读取，会导致没有/joint_states数据发布，rviz中的模型的所有link会在原点聚集，无法正常显示**

如果不想启动实机机械臂数据读取节点，可以只启动rviz的显示launch，然后在其他程序中或者终端发布`joint_states`topic，示例如下

```bash
# 有夹爪
ros2 topic pub /joint_states sensor_msgs/msg/JointState "header:
  stamp:
    sec: 1757669668
    nanosec: 196177482
  frame_id: ''
name: ['joint1','joint2','joint3','joint4','joint5','joint6','gripper','joint7','joint8']
position: [0,0,0,0,0,0,0,0,0]
velocity: [0,0,0,0,0,0,0,0,0]
effort: [0,0,0,0,0,0,0,0,0]" 
```

```bash
# 无夹爪
ros2 topic pub /joint_states sensor_msgs/msg/JointState "header:
  stamp:
    sec: 1757669668
    nanosec: 196177482
  frame_id: ''
name: ['joint1','joint2','joint3','joint4','joint5','joint6']
position: [0,0,0,0,0,0]
velocity: [0,0,0,0,0,0]
effort: [0,0,0,0,0,0]" 
```

注意：**header中的时间戳为必需，否则会导致rviz显示的模型依旧不正常**

### 3.3 同时控制双臂

需要每条臂各自链接一个can模块，然后激活两个can模块

launch内读取的两个can模块名称都为`can0`，需要自行手动修改为左右臂在系统中的名称，查看方法如下：

```bash
ifconfig -a | grep can
```

执行双臂启动

```bash
# 默认是启动两个节点都读取can0消息
ros2 launch piper start_two_piper.launch.py auto_enable:=false
# 自定义读取的can端口名称
ros2 launch piper start_two_piper.launch.py can_left_port:=can_left can_right_port:=can_right auto_enable:=true girpper_exist:=true
```

- **launch内部做了topic的remap**

```python
# piper_left_node
remappings=[
            # 控制
            ('pos_cmd', '/pos_cmd_left'),
            ('joint_ctrl_single', '/joint_ctrl_cmd_left'),
            # 反馈
            ('joint_states_single', '/joint_states_left'),
            ('joint_states_feedback', '/joint_left'),
            ('joint_ctrl', '/joint_states_ctrl_left'),
            ('arm_status', '/arm_status_left'),
            ('end_pose', '/end_pose_left'),
            ('end_pose_stamped', '/end_pose_stamped_left'),
        ]
# piper_right_node
remappings=[
            # 控制
            ('pos_cmd', '/pos_cmd_right'),
            ('joint_ctrl_single', '/joint_ctrl_cmd_right'),
            # 反馈
            ('joint_states_single', '/joint_states_right'),
            ('joint_states_feedback', '/joint_right'),
            ('joint_ctrl', '/joint_states_ctrl_right'),
            ('arm_status', '/arm_status_right'),
            ('end_pose', '/end_pose_right'),
            ('end_pose_stamped', '/end_pose_stamped_right'),
        ]
```

对照上文单节点的topic说明对应remap后的topic含义

```bash
/arm_status_left
/arm_status_right
/enable_flag
/end_pose_left
/end_pose_right
/end_pose_stamped_left
/end_pose_stamped_right
/joint_ctrl_cmd_left
/joint_ctrl_cmd_right
/joint_left
/joint_right
/joint_states_ctrl_left
/joint_states_ctrl_right
/joint_states_left
/joint_states_right
/pos_cmd_left
/pos_cmd_right
```

### 3.4 piper自定义消息

ros功能包`piper_msgs`

机械臂自身状态反馈消息，对应can协议中`id=0x2A1`的反馈消息

`PiperStatusMsg.msg`

```c
uint8 ctrl_mode
/*
0x00 待机模式  
0x01 CAN指令控制模式
0x02 示教模式
0x03 以太网控制模式
0x04 wifi控制模式
0x05 遥控器控制模式
0x06 联动示教输入模式
0x07 离线轨迹模式*/
uint8 arm_status
/*
0x00 正常
0x01 急停
0x02 无解
0x03 奇异点
0x04 目标角度超过限
0x05 关节通信异常
0x06 关节抱闸未打开 
0x07 机械臂发生碰撞
0x08 拖动示教时超速
0x09 关节状态异常
0x0A 其它异常  
0x0B 示教记录
0x0C 示教执行
0x0D 示教暂停
0x0E 主控NTC过温
0x0F 释放电阻NTC过温*/
uint8 mode_feedback
/*
0x00 MOVE P
0x01 MOVE J
0x02 MOVE L
0x03 MOVE C*/
uint8 teach_status
/*
0x00 关闭
0x01 开始示教记录（进入拖动示教模式）
0x02 结束示教记录（退出拖动示教模式）
0x03 执行示教轨迹（拖动示教轨迹复现）
0x04 暂停执行
0x05 继续执行（轨迹复现继续）
0x06 终止执行
0x07 运动到轨迹起点*/
uint8 motion_status
/*
0x00 到达指定点位
0x01 未到达指定点位*/
uint8 trajectory_num
/*0~255 (离线轨迹模式下反馈)*/
int64 err_code//故障码
bool joint_1_angle_limit//1号关节通信异常（0：正常 1：异常）
bool joint_2_angle_limit//2号关节通信异常（0：正常 1：异常）
bool joint_3_angle_limit//3号关节通信异常（0：正常 1：异常）
bool joint_4_angle_limit//4号关节通信异常（0：正常 1：异常）
bool joint_5_angle_limit//5号关节通信异常（0：正常 1：异常）
bool joint_6_angle_limit//6号关节通信异常（0：正常 1：异常）
bool communication_status_joint_1//1号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_2//2号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_3//3号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_4//4号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_5//5号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_6//6号关节角度超限位（0：正常 1：异常）
```

机械臂末端位姿控制，注意:有些奇异点无法到达

`PosCmd.msg`

```c
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 gripper
int32 mode1
int32 mode2
```

## 注意事项

- 需要先激活can设备，并且设置正确的波特率，才可以读取机械臂消息或者控制机械臂
- 如果出现

  ```shell
  使能状态: False
  <class 'can.exceptions.CanError'> Message NOT sent
  <class 'can.exceptions.CanError'> Message NOT sent
  ```

  说明机械臂没有与can模块连同，拔插usb后，重新启动机械臂，再激活can模块，然后尝试重新启动节点

- 如果打开了自动使能，尝试使能5s没有成功后，会自动退出程序

## Q&A

1. 出现

```shell
colcon build
Starting >>> piper   
Starting >>> piper_control
Starting >>> piper_description
Starting >>> piper_msgs
Finished <<< piper [0.31s]                                                                                                              
--- stderr: piper_description                                                                                            
CMake Error at CMakeLists.txt:5 (find_package):
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.

  Could not find a package configuration file provided by "ament_cmake" with
  any of the following names:

    ament_cmakeConfig.cmake
    ament_cmake-config.cmake

  Add the installation prefix of "ament_cmake" to CMAKE_PREFIX_PATH or set
  "ament_cmake_DIR" to a directory containing one of the above files.  If
  "ament_cmake" provides a separate development package or SDK, be sure it
  has been installed.


---
Failed   <<< piper_description [0.45s, exited with code 1]
Aborted  <<< piper_control [0.45s]
Aborted  <<< piper_msgs [0.45s]

Summary: 1 package finished [0.55s]
  1 package failed: piper_description
  2 packages aborted: piper_control piper_msgs
  3 packages had stderr output: piper_control piper_description piper_msgs
```

```shell
source /opt/ros/$ROS_DISTRO/setup.bash
```

2. 

```shell
Starting >>> piper   
Starting >>> piper_control
Starting >>> piper_description
Starting >>> piper_msgs
Finished <<< piper [0.33s]                                                                                                              
Finished <<< piper_description [0.42s]                                                                                     
--- stderr: piper_control                                                               
CMake Error at CMakeLists.txt:7 (find_package):
  By not providing "Findcontroller_manager.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "controller_manager", but CMake did not find one.

  Could not find a package configuration file provided by
  "controller_manager" with any of the following names:

    controller_managerConfig.cmake
    controller_manager-config.cmake

  Add the installation prefix of "controller_manager" to CMAKE_PREFIX_PATH or
  set "controller_manager_DIR" to a directory containing one of the above
  files.  If "controller_manager" provides a separate development package or
  SDK, be sure it has been installed.


---
Failed   <<< piper_control [0.80s, exited with code 1]
Aborted  <<< piper_msgs [3.28s]                               

Summary: 2 packages finished [3.38s]
  1 package failed: piper_control
  1 package aborted: piper_msgs
  1 package had stderr output: piper_control
```

```shell
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

3. 

出现：

```shell
 "package 'joint_state_publisher_gui' not found
```

安装：

```shell
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
```

4. 

```shell
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
```

5. 

```shell
sudo apt install ros-$ROS_DISTRO-xacro
```
