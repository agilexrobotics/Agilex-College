# 从零到玩转Moveit机械臂控制（一）

## 摘要

使用URDF导出Moveit功能包

## 标签

ROS1、moveit、机械臂

## 仓库

- **导航仓库**: https://github.com/agilexrobotics/Agilex-College
- **项目仓库**: https://github.com/agilexrobotics/piper/piper_moveit.git

## 使用环境

系统：ubuntu 20.04

ROS版本：noetic

## MoveIt简介

**MoveIt 是 ROS1 中最成熟、应用最广泛的机械臂运动规划框架**，为机器人提供从模型描述到运动执行的一整套解决方案。

MoveIt 主要解决的问题包括：

- 机械臂运动规划（Motion Planning）
- 碰撞检测（Collision Checking）
- 逆运动学（IK）
- 轨迹生成与执行
- RViz 可视化与交互

## Moveit安装

可以直接使用二进制安装；使用以下命令安装和moveit相关的所有组件

```
sudo apt install ros-noetic-moveit*
```

## 下载URDF文件

首先创建一个新的工作空间；并下载URDF 模型

```
mkdir -p ~/piper_ws/src
cd ~/piper_ws/src
catkin_init_workspace
git clone https://github.com/smalleha/piper_urdf -b noetic
cd ..
catkin_make
```

编译通过之后，可以使用以下命令在rviz中查看模型

> 这里以piper_x作例子

```
cd ~/piper_ws/src
source devel/setup.bash
roslaunch piper_x_description display_xacro.launch
```

![](./img/img_1.png)

## 使用Setup Assistant导出Moveit包

启动moveit_setup_assistant

```
roslaunch moveit_setup_assistant setup_assistant.launch
```

![](./img/img_2.png)

选择Create New Moveit Configuration Package；创建一个新的Moveit功能包；然后加载机械臂

![](./img/img_3.png)

计算碰撞模型；因为自由一条臂，使用默认参数即可

![](./img/img_4.png)

跳过选择虚拟关节；来到定义规划组；我们这里需要创建两个规划组，机械臂规划组和夹爪规划组；这里先创建机械臂规划组；Group Name设为arm，运动学求解器使用KDL，OMPL Planning 选择RRTstar

![](./img/img_5.png)

设置Kin.Chain

![](./img/img_6.png)

添加规划组的控制关节，选择joint1~joint6，点击>，然后保存

![](./img/img_7.png)

创建gripper的规划组，步骤略有不同；因为夹爪的只有两个关节，所以不需要用到运动学求解器和OMPL

![](./img/img_8.png)

 添加规划组的控制关节，选择joint7~joint8，点击>，然后保存

![](./img/img_9.png)

规划组创建完成

![](./img/img_10.png)

设置Robot Pose；在这里可以预先设置好规划组的一些动作；因为是二指夹爪，必须得设置夹爪打开和闭合的动作

![](./img/img_11.png)

先设置机械臂的

![](./img/img_12.png)

设置夹爪闭合状态

![](./img/img_13.png)

设置夹爪张开状态

![](./img/img_14.png)

跳过End Effectors和Passive Joints；设置控制器，这里使用的是position_controllers

![](./img/img_15.png)

Simulation 会生成和gazebo中使用的URDF文件，里面添加了关节的电机属性等物理属性

![](./img/img_16.png)

配置完之后，填上自己的名称以及邮箱

![](./img/img_17.png)

设置功能包名称；然后点击Generate Package输出功能包

![](./img/img_18.png)

## 启动moveit功能包

```
cd ~/piper_ws/src
source devel/setup.bash
roslaunch piper_x_moveit_config demo.launch
```

![](./img/img_19.png)

成功启动之后，就可以拖动悬浮球，预设机械臂的位置，点击Plan & Execute控制机械臂运动了
