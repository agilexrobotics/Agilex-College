## **【标题】LIMO PRO实现Mujoco仿真**
| | 作者 | 复核 | 排版 | 发布 |
| --- | :---: | :---: | :---: | :---: |
| 人   员 | Khalil | XXX（英文名） | XXX（英文名） | XXX（英文名） |
| 更新时间 | 2025年8月5日 | 2025年XX月XX日 | 2025年XX月XX日 | 2025年XX月XX日 |


## 摘要
本章将会一步步展示Limo Pro底盘从处理URDF、导出Mujoco XML格式、仿真数据调试最后到mujoco自定义控制器的完整实现步骤。

## 标签
Mujoco仿真、Mujoco控制器、URDF、松灵Limo

## 代码仓库
github链接：[**https://github.com/agilexrobotics/Agilex-College.git**](https://github.com/agilexrobotics/Agilex-College.git)

## 功能演示
[此处为语雀卡片，点击链接查看](https://www.yuque.com/docs/231105255#OurNT)

---

# 使用前准备
## 硬件准备
+ 松灵底盘系列任选一款
+ 个人电脑

## 软件环境配置
1. 配置Mujoco环境

```bash
# 创建mujoco默认路径
cd ~
mkdir .mujoco
cd .mujoco
# 下载mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz
# 解压
tar -zxvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco
# 添加环境变量
echo "export LD_LIBRARY_PATH=~/.mujoco/mujoco200/bin${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/dlut/.mujoco/mujoco200/bin" >> ~/.bashrc
echo "export MUJOCO_KEY_PATH=~/.mujoco${MUJOCO_KEY_PATH}" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
```

2. 测试mujoco是否安装正常

```bash
cd ~/.mujoco/mujoco2000/bin
./simulate ../model/humanoid.xml
```

3. 安装mujoco-py

```bash
git clone https://github.com/openai/mujoco-py.git
cd ~/mujoco-py
pip3 install -U 'mujoco-py<2.2,>=2.1'
pip3 install -r requirements.txt
pip3 install -r requirements.dev.txt
python3 setup.py install
sudo apt install libosmesa6-dev
sudo apt install patchelf
```

4. 测试mujoco-py是否安装正常

```python
import mujoco_py
import os
mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'humanoid.xml')
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
print(sim.data.qpos)
sim.step()
print(sim.data.qpos)
```

5. 获取Limo pro的URDF

```bash
cd your_ws/src
git clone https://github.com/agilexrobotics/ugv_gazebo_sim.git
cd ..
catkin_make
```

---

# 准备Mujoco XML
## 转换XACRO为URDF
1. 观察Limo pro的描述文件以及模型文件，因为描述文件全部用xacro组织的，所有需要将描述文件转换为一个包含描述完整机器人的URDF文件

```bash
urdf/
├── limo_four_diff.gazebo
├── limo_four_diff.xacro
├── limo_gazebo.gazebo
├── limo_steering_hinge.xacro
└── limo_xacro.xacro
meshes/
├── limo_base.dae
├── limo_base.stl
├── limo_wheel.dae
└── limo_wheel.stl
```

2. 找到最顶层的xacro文件，本期使用的Limo Pro为`limo_four_diff.xacro`
3. 然后执行`xacro-->>urdf`的转换命令

```bash
rosrun xacro xacro limo_four_diff.xacro > limo_four_diff.urdf 
```

4. 修改转换后的`limo_four_diff.urdf`，主要是<font style="background-color:#FBDE28;">去掉其中的gazebo插件</font>，还需要<font style="background-color:#FBDE28;">将加载DAE模型改为加载STL模型文件</font>

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754386067183-48698297-3b84-4d1f-97f2-9b265d0cdbb3.png)

## 转换DEA到STL并减少模型面数
1. Mujoco最大支持单个模型19999个面，对于Limo pro的模型我们需要先利用`meshlab`处理模型

```bash
# 安装meshlab
sudo apt install meshlab
```

2. 启动meshlab修改模型，执行下面的命令后会弹出meshlab的操作界面

```bash
meshlab
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754387288860-e1be202d-e4cc-461d-8d3b-427384fd8fdf.png)

3. 点击左上角第一个按钮`File`，选择`Import Mesh`按钮或者按`ctrl+i`导入dea文件
4. 导入dea后，选择左上角第三个`Filter`，展开后选择`Remeshing，Simplification and Reconstruction`，再展开后选择`Simplification Quadric Edge Collapse Decimation`

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754387662169-c43df37b-ae06-4fcd-aad0-cd63672e98c0.png)

5. 然后输入目标面数，我们需要减少到mujoco接受的19999,然后点击`Apply`

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754387776248-87edc270-cdcc-4802-b45d-800eca57e4c9.png)

6. 然后再次点击左上角第一个按钮`File`，选择`Export Mesh as...`后会弹出文件管理器，在文件名后面修改文件格式后缀为`.stl`

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754388083214-3dd92454-379b-453c-9283-dc0436aa4b3b.png)

7. 按上面的操作再处理车轮的模型文件

## 转换URDF为Mujoco XML
1. 使用Mujoco提供的工具来转换，工具路径为：

```bash
~/.mujoco/mujoco210/bin/compile
```

2. 使用转换工具转换为Mujoco XML

```bash
./compile ~/your_ws/src/package/urdf/limo_four_diff.urdf ~/your_ws/src/package/urdf/limo_four_diff.xml
```

3. 转换成功后可以看到下面的文件

```xml
<mujoco model="limo_four_diff">
  <compiler angle="radian" meshdir="/home/khalillee/sim1_ws/src/limo/limo_description/meshes/" />
  <size njmax="500" nconmax="100" />
  <asset>
    <mesh name="limo_base_dae2stl" file="limo_base_dae2stl.stl" />
    <mesh name="limo_wheel_dae2stl" file="limo_wheel_dae2stl.stl" />
  </asset>
  <worldbody>
    <geom quat="0.707388 0 0 0.706825" type="mesh" contype="0" conaffinity="0" group="1" mesh="limo_base_dae2stl" />
    <geom size="0.065 0.06 0.05" pos="0 0 0.15" type="box" />
    <geom size="0.02 0.005" pos="0.103 0 0.116" type="cylinder" contype="0" conaffinity="0" group="1" />
    <geom size="0.032 0.008" pos="0.103 0 0.116" type="cylinder" />
    <geom size="0.01 0.03 0.0075" pos="0.084 0 0.18" type="box" contype="0" conaffinity="0" group="1" />
    <geom size="0.01 0.03 0.0075" pos="0.084 0 0.18" type="box" />
    <geom size="0.0005 0.0005 0.0005" pos="0 0 0.05" type="box" contype="0" conaffinity="0" group="1" />
    <geom size="0.0005 0.0005 0.0005" pos="0 0 0.05" type="box" />
    <body name="front_left_wheel_link" pos="0.1 0.065 0.05">
      <inertial pos="0 0 0" quat="0.707107 0.707107 0 0" mass="0.5" diaginertia="0.01055 0.01055 0.00075" />
      <joint name="front_left_wheel" pos="0 0 0" axis="0 1 0" />
      <geom type="mesh" contype="0" conaffinity="0" group="1" mesh="limo_wheel_dae2stl" />
      <geom size="0.045 0.0225" pos="0 0.0225 0" quat="0.707388 0.706825 0 0" type="cylinder" />
    </body>
    <body name="front_right_wheel_link" pos="0.1 -0.065 0.05" quat="1.32679e-06 1 0 0">
      <inertial pos="0 0 0" quat="0.707107 0.707107 0 0" mass="0.5" diaginertia="0.01055 0.01055 0.00075" />
      <joint name="front_right_wheel" pos="0 0 0" axis="0 -1 0" />
      <geom type="mesh" contype="0" conaffinity="0" group="1" mesh="limo_wheel_dae2stl" />
      <geom size="0.045 0.0225" pos="0 0.0225 0" quat="0.707388 0.706825 0 0" type="cylinder" />
    </body>
    <body name="rear_left_wheel_link" pos="-0.1 0.065 0.05">
      <inertial pos="0 0 0" quat="0.707107 0.707107 0 0" mass="0.5" diaginertia="0.01055 0.01055 0.00075" />
      <joint name="rear_left_wheel" pos="0 0 0" axis="0 1 0" />
      <geom type="mesh" contype="0" conaffinity="0" group="1" mesh="limo_wheel_dae2stl" />
      <geom size="0.045 0.0225" pos="0 0.0225 0" quat="0.707388 0.706825 0 0" type="cylinder" />
    </body>
    <body name="rear_right_wheel_link" pos="-0.1 -0.065 0.05" quat="1.32679e-06 1 0 0">
      <inertial pos="0 0 0" quat="0.707107 0.707107 0 0" mass="0.5" diaginertia="0.01055 0.01055 0.00075" />
      <joint name="rear_right_wheel" pos="0 0 0" axis="0 -1 0" />
      <geom type="mesh" contype="0" conaffinity="0" group="1" mesh="limo_wheel_dae2stl" />
      <geom size="0.045 0.0225" pos="0 0.0225 0" quat="0.707388 0.706825 0 0" type="cylinder" />
    </body>
    </worldbody>
</mujoco>

```

4. 检查转换是否正确，如果转换成功可以看到Mujoco仿真界面

```xml
./simulate ~/your_ws/src/limo/limo_description/urdf/limo_four_diff.xml
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754385871618-a1d18326-a363-4750-8f1f-436d8f0da164.png)

## 完善Mujoco XML以仿真
1. 此时的Mujoco XML只能查看模型，我们无法控制它，所以需要为XML添加动作执行器和传感器，下面展示完善后的XML文件

```xml
<mujoco model="limo_four_diff">
  <compiler angle="radian" meshdir="/home/khalillee/sim1_ws/src/limo/limo_description/meshes/" />
  <size njmax="500" nconmax="100" />
  <asset>
    <mesh name="limo_base_dae2stl" file="limo_base_dae2stl.stl" />
    <mesh name="limo_wheel_dae2stl" file="limo_wheel_dae2stl.stl" />
  </asset>

  <!-- 全局物理参数 -->
  <option timestep="0.005"/>
  <option cone="elliptic"/>
  <option impratio="0.5"/>
  <option integrator="RK4"/>
  <!-- Add air resistance -->
  <option wind="0 0 0" density="1.2" viscosity="1.8e-5"/>
  <!-- Enable gravity compensation -->
  <option gravity="0 0 -9.81"/>

  <worldbody>
    <!-- 地面 (添加摩擦参数) -->
    <geom name="ground" type="plane" size="10 10 0.1" rgba="0.8 0.9 0.8 1"
      solimp="0.9 0.99 0.001" solref="0.01 1" friction="1.4 0.06 0.005" solmix="0.95"/>
    <body name="base_link" pos="0 0 0.15">
      <freejoint name="base_free_joint"/>
      <geom quat="0.707388 0 0 0.706825" type="mesh" contype="0" conaffinity="0" group="1" mesh="limo_base_dae2stl" mass="3" rgba="0.3 0.5 0.8 1" friction="0.8 0.3 0.1"/>
      <!-- <geom size="0.065 0.06 0.05" pos="0 0 0.15" type="box" /> -->
      <!-- <geom size="0.02 0.005" pos="0.103 0 0.116" type="cylinder" contype="0" conaffinity="0" group="1" />
      <geom size="0.032 0.008" pos="0.103 0 0.116" type="cylinder" />
      <geom size="0.01 0.03 0.0075" pos="0.084 0 0.18" type="box" contype="0" conaffinity="0" group="1" />
      <geom size="0.01 0.03 0.0075" pos="0.084 0 0.18" type="box" />
      <geom size="0.0005 0.0005 0.0005" pos="0 0 0.05" type="box" contype="0" conaffinity="0" group="1" />
      <geom size="0.0005 0.0005 0.0005" pos="0 0 0.05" type="box" /> -->

      <body name="front_left_wheel_link" pos="0.1 0.065 0.05">
        <inertial pos="0 0 0" quat="0.707107 0.707107 0 0" mass="0.5" diaginertia="0.01055 0.01055 0.00075" />
        <joint name="front_left_wheel" type="hinge" pos="0 0 0" axis="0 1 0" damping="0.1"/>
        <geom type="mesh" mesh="limo_wheel_dae2stl" mass="0.5" 
          rgba="0.1 0.1 0.1 1" friction="1.5 0.1 0.005" solimp="0.9 0.95 0.001" solref="0.01 1"
          contype="1" conaffinity="1"/>
        <!-- <geom size="0.05 0.0225" pos="0 0.0225 0" quat="0.707388 0.706825 0 0" type="cylinder" friction="1.2 0.005 0.0001"
        solimp="0.9 0.95 0.001" solref="0.02 1"/> -->
      </body>
      <body name="front_right_wheel_link" pos="0.1 -0.065 0.05" quat="1.32679e-06 1 0 0">
        <inertial pos="0 0 0" quat="0.707107 0.707107 0 0" mass="0.5" diaginertia="0.01055 0.01055 0.00075" />
        <joint name="front_right_wheel" type="hinge" pos="0 0 0" axis="0 -1 0" damping="0.1"/>
        <geom type="mesh" mesh="limo_wheel_dae2stl" mass="0.5" 
          rgba="0.1 0.1 0.1 1" friction="1.5 0.1 0.005" solimp="0.9 0.95 0.001" solref="0.01 1"
                      contype="1" conaffinity="1"/>
                <!-- <geom size="0.05 0.0225" pos="0 0.0225 0" quat="0.707388 0.706825 0 0" type="cylinder" friction="1.2 0.005 0.0001"
                        solimp="0.9 0.95 0.001" solref="0.02 1"/> -->
            </body>
            <body name="rear_left_wheel_link" pos="-0.1 0.065 0.05">
                <inertial pos="0 0 0" quat="0.707107 0.707107 0 0" mass="0.5" diaginertia="0.01055 0.01055 0.00075" />
                <joint name="rear_left_wheel" type="hinge" pos="0 0 0" axis="0 1 0" damping="0.1"/>
                <geom type="mesh" mesh="limo_wheel_dae2stl" mass="0.5" 
                      rgba="0.1 0.1 0.1 1" friction="1.5 0.1 0.005" solimp="0.9 0.95 0.001" solref="0.01 1"
                      contype="1" conaffinity="1"/>
                <!-- <geom size="0.05 0.0225" pos="0 0.0225 0" quat="0.707388 0.706825 0 0" type="cylinder" friction="1.2 0.005 0.0001"
                        solimp="0.9 0.95 0.001" solref="0.02 1"/> -->
            </body>
            <body name="rear_right_wheel_link" pos="-0.1 -0.065 0.05" quat="1.32679e-06 1 0 0">
                <inertial pos="0 0 0" quat="0.707107 0.707107 0 0" mass="0.5" diaginertia="0.01055 0.01055 0.00075" />
                <joint name="rear_right_wheel" type="hinge" pos="0 0 0" axis="0 -1 0" damping="0.1"/>
                <geom type="mesh" mesh="limo_wheel_dae2stl" mass="0.5" 
                      rgba="0.1 0.1 0.1 1" friction="1.5 0.1 0.005" solimp="0.9 0.95 0.001" solref="0.01 1"
                      contype="1" conaffinity="1"/>
                <!-- <geom size="0.05 0.0225" pos="0 0.0225 0" quat="0.707388 0.706825 0 0" type="cylinder" friction="1.2 0.005 0.0001"
                        solimp="0.9 0.95 0.001" solref="0.02 1"/> -->
            </body>
        </body>
    </worldbody>
    <actuator>
        <!-- 前轮驱动 -->
        <motor name="left_f_wheel_motor" joint="front_left_wheel" gear="0.5" ctrllimited="true" ctrlrange="-10 10"
           forcelimited="true" forcerange="-5 5"/>
        <motor name="right_f_wheel_motor" joint="front_right_wheel" gear="0.5" ctrllimited="true" ctrlrange="-10 10"
           forcelimited="true" forcerange="-5 5"/>
        <!-- 后轮驱动 -->
        <motor name="left_b_wheel_motor" joint="rear_left_wheel" gear="0.5" ctrllimited="true" ctrlrange="-10 10"
           forcelimited="true" forcerange="-5 5"/>
        <motor name="right_b_wheel_motor" joint="rear_right_wheel" gear="0.5" ctrllimited="true" ctrlrange="-10 10"
           forcelimited="true" forcerange="-5 5"/>
    </actuator>
    <sensor>
        <jointvel name="front_left_wheel_vel_sensor" joint="front_left_wheel" />
        <jointvel name="front_right_wheel_vel_sensor" joint="front_right_wheel" />
        <jointvel name="rear_left_wheel_vel_sensor" joint="rear_left_wheel" />
        <jointvel name="rear_right_wheel_vel_sensor" joint="rear_right_wheel" />
    </sensor>
</mujoco>

```

2. 再次启动仿真程序，拖动Control面板的滑条控制小车移动

```xml
./simulate limo/limo_description/urdf/limo_four_diff.xml
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754388705858-c4dacec4-fb16-499b-8e05-786211b1d41b.png)

# 编写Mujoco控制器与ROS接口
## 完成程序代码
```python
#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
import math
from pynput import keyboard
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class LimoControl:
    def __init__(self):
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path('/home/khalillee/sim1_ws/src/limo/limo_description/urdf/test.xml')
        self.data = mujoco.MjData(self.model)
        
        # 控制参数
        self.MAX_SPEED = 10.0  # 最大速度 (rad/s)
        self.LINEAR_VEL = 2.0  # 线性速度 (m/s)
        self.ANGULAR_VEL = 10.0  # 角速度 (rad/s)
        
        # ROS参数
        self.use_ros_control = rospy.get_param('~use_ros_control', False)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.05)
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.13)
        self.base_width = rospy.get_param('~base_width', 0.2)
        
        # 初始化ROS节点
        rospy.init_node('limo_mujoco_control')
        
        if self.use_ros_control:
            self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
            rospy.loginfo("使用ROS cmd_vel控制模式")
        else:
            self.start_keyboard_listener()
            rospy.loginfo("使用键盘控制模式")
        
        # 初始化传感器数据发布
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [
            'front_left_wheel', 
            'front_right_wheel',
            'rear_left_wheel',
            'rear_right_wheel'
        ]
        
        # 控制变量
        self.ros_cmd_vel = None
        self.current_keys = set()
        
    def cmd_vel_callback(self, msg):
        """ROS cmd_vel回调函数"""
        self.ros_cmd_vel = msg
        
    def set_motor_velocity(self, vel_fl, vel_fr, vel_rl, vel_rr):
        """设置四个电机的速度"""
        self.data.ctrl[0] = vel_fl  # 左前轮
        self.data.ctrl[1] = vel_fr  # 右前轮
        self.data.ctrl[2] = vel_rl  # 左后轮
        self.data.ctrl[3] = vel_rr  # 右后轮
        
    def stop(self):
        """停止"""
        self.set_motor_velocity(0, 0, 0, 0)
        
    def differential_drive(self, v, w):
        """
        差速驱动控制
        v: 线速度 (m/s)
        w: 角速度 (rad/s)
        """
        # 计算四个轮子的速度 (rad/s)
        vel_fl = (v - w * (self.wheel_separation + self.base_width)/2) / self.wheel_radius
        vel_fr = (v + w * (self.wheel_separation + self.base_width)/2) / self.wheel_radius
        vel_rl = (v - w * (self.wheel_separation + self.base_width)/2) / self.wheel_radius
        vel_rr = (v + w * (self.wheel_separation + self.base_width)/2) / self.wheel_radius
        
        # 限制最大速度
        vel_fl = np.clip(vel_fl, -self.MAX_SPEED, self.MAX_SPEED)
        vel_fr = np.clip(vel_fr, -self.MAX_SPEED, self.MAX_SPEED)
        vel_rl = np.clip(vel_rl, -self.MAX_SPEED, self.MAX_SPEED)
        vel_rr = np.clip(vel_rr, -self.MAX_SPEED, self.MAX_SPEED)
        
        self.set_motor_velocity(vel_fl, vel_fr, vel_rl, vel_rr)
        
    def publish_joint_states(self):
        """发布关节状态"""
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.header.frame_id = "base_link"
        
        # 从模型获取关节位置
        self.joint_state_msg.position = [
            self.data.joint('front_left_wheel').qpos[0],
            self.data.joint('front_right_wheel').qpos[0],
            self.data.joint('rear_left_wheel').qpos[0],
            self.data.joint('rear_right_wheel').qpos[0]
        ]
        
        # 从传感器获取轮速
        self.joint_state_msg.velocity = [
            self.data.sensor('front_left_wheel_vel_sensor').data[0],
            self.data.sensor('front_right_wheel_vel_sensor').data[0],
            self.data.sensor('rear_left_wheel_vel_sensor').data[0],
            self.data.sensor('rear_right_wheel_vel_sensor').data[0]
        ]
        
        # 发布关节状态
        self.joint_state_pub.publish(self.joint_state_msg)
        
    def on_press(self, key):
        """键盘按下事件"""
        try:
            self.current_keys.add(key.char)
        except AttributeError:
            self.current_keys.add(key)
            
    def on_release(self, key):
        """键盘释放事件"""
        try:
            self.current_keys.remove(key.char)
        except AttributeError:
            try:
                self.current_keys.remove(key)
            except KeyError:
                pass
                
    def handle_keyboard_control(self):
        """处理键盘控制"""
        linear = 0.0
        angular = 0.0
        
        if '8' in self.current_keys or keyboard.Key.up in self.current_keys:
            linear += self.LINEAR_VEL
        if '2' in self.current_keys or keyboard.Key.down in self.current_keys:
            linear -= self.LINEAR_VEL
        if '4' in self.current_keys or keyboard.Key.left in self.current_keys:
            angular += self.ANGULAR_VEL
        if '6' in self.current_keys or keyboard.Key.right in self.current_keys:
            angular -= self.ANGULAR_VEL
        if ' ' in self.current_keys:  # 空格键停止
            linear = 0.0
            angular = 0.0
            
        self.differential_drive(linear, angular)
        
    def start_keyboard_listener(self):
        """启动键盘监听"""
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        rospy.loginfo("键盘控制已启用: 8/↑: 前进, 2/↓: 后退, 4/←: 左转, 6/→: 右转, 空格: 停止")
        
    def run(self):
        """主运行循环"""
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while not rospy.is_shutdown() and viewer.is_running():
                    step_start = time.time()
                    
                    # 处理控制输入
                    if self.use_ros_control and self.ros_cmd_vel is not None:
                        self.differential_drive(self.ros_cmd_vel.linear.x, self.ros_cmd_vel.angular.z)
                    else:
                        self.handle_keyboard_control()
                    
                    # 步进模拟
                    mujoco.mj_step(self.model, self.data)
                    
                    # 发布传感器数据
                    self.publish_joint_states()
                    
                    # 同步视图
                    viewer.sync()
                    
                    # 控制循环频率
                    time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                        
        except KeyboardInterrupt:
            rospy.loginfo("Simulation stopped by user")
        finally:
            self.stop()
            rospy.loginfo("Final robot state published")

if __name__ == "__main__":
    control = LimoControl()
    control.run()
```

## 启动控制器
1. 启动Mujoco控制器

```python
rosrun limo_description test_mujoco.py _use_ros_control:=False
```

2. 或者启动ROS控制器

```python
rosrun limo_description test_mujoco.py _use_ros_control:=True
```

3. 启动成功后可以看到Mojuco界面，使用<font style="background-color:#FBDE28;">上下左右键</font>或者<font style="background-color:#FBDE28;">8、2、4、6键</font>进行控制如果使用ROS控制器，还需要启动<font style="background-color:#FBDE28;">键盘控制节点</font>

```python
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

4. 在RVIZ中同步小车车轮运动

```python
roslaunch limo_description display_models.launch 
```

