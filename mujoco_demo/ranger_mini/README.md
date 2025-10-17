## **【标题】Ranger Mini四轮四转底盘实现Mujoco仿真与PID控制器**
| | 作者 | 复核 | 排版 | 发布 |
| --- | :---: | :---: | :---: | :---: |
| 人   员 | Khalil | XXX（英文名） | XXX（英文名） | XXX（英文名） |
| 更新时间 | 2025年8月5日 | 2025年XX月XX日 | 2025年XX月XX日 | 2025年XX月XX日 |


## 摘要
本章将会一步步展示Ranger Mini四轮四转底盘从处理URDF、导出Mujoco XML格式、仿真数据调试最后到mujoco中实现PID控制器的完整实现步骤。

## 标签
Mujoco仿真、Mujoco控制器、URDF、舵轮、PID控制、松灵Ranger Mini

## 代码仓库
github链接：[**https://github.com/agilexrobotics/Agilex-College.git**](https://github.com/agilexrobotics/Agilex-College.git)

## 功能演示
[此处为语雀卡片，点击链接查看](https://www.yuque.com/docs/231440707#mXRfw)



# 使用前准备
## 硬件准备
+ 松灵Ranger Mini
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

5. 获取Ranger mini的URDF以及STL模型

```bash
cd your_ws/src
git clone https://github.com/agilexrobotics/ugv_gazebo_sim.git
cd ..
catkin_make
```

---

# 准备Mujoco XML
## 转换XACRO为URDF
1. 观察Ranger mini的描述文件以及模型文件，因为描述文件用xacro组织的，所有需要将描述文件转换为一个包含描述完整机器人的URDF文件

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754559114732-ceeb775d-5928-4dd8-88a8-e636b81862fa.png)

```bash
meshes/
├── ranger_base.dae
├── ranger_base.zip
├── steering_wheel.dae
└──  wheel_v3.dae
urdf/
├── ranger_mini.csv
└── ranger_mini.xacro
```

2. 找到最顶层的xacro文件，本期使用的Ranger mini为`ranger_mini.xacro`
3. 然后执行`xacro-->>urdf`的转换命令

```bash
mkdir mujoco_model
cd mujoco_model
rosrun xacro xacro ranger_mini.xacro > ranger_mini.urdf 
```

4. 修改转换后的`limo_four_diff.urdf`，首先添加mujoco指定模型文件路径的标签，注意修改为自己模型的路径

```xml
<mujoco>
    <compiler balanceinertia="true" discardvisual="false" meshdir="/home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/meshes"/>
</mujoco>
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754559394633-685da92f-5bed-4126-a30d-109ce326ab63.png)

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

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754559523413-28451b10-e6d5-447a-a528-ca74821bcded.png)

3. 如图所示点击左上角按钮，选择`Import Mesh`按钮或者按`ctrl+i`导入dea文件
4. 导入dea后，<font style="background-color:#FBDE28;">选择左上角第三个</font>`<font style="background-color:#FBDE28;">Filter</font>`<font style="background-color:#FBDE28;">，展开后选择</font>`<font style="background-color:#FBDE28;">Remeshing，Simplification and Reconstruction</font>`<font style="background-color:#FBDE28;">，再展开后选择</font>`<font style="background-color:#FBDE28;">Simplification Quadric Edge Collapse Decimation</font>`

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754559627778-b3feeb3d-f2e6-4465-810f-280a2c2f185f.png)

5. 然后输入目标面数，我们需要减少到mujoco接受的19999,然后点击`Apply`

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754559665528-303f8705-cc9e-4e1b-9699-7d7fae2354e1.png)

6. 然后再次点击左上角第一个按钮`File`，选择`Export Mesh as...`后会弹出文件管理器，在文件名后面修改文件格式后缀为`.stl`

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754559722652-16b816e6-1a36-4c15-9000-d5a8d8ae67d3.png)

7. <font style="background-color:#FBDE28;">按上面的操作再处理车轮的模型文件</font>

## 转换URDF为Mujoco XML
1. 使用Mujoco提供的工具来转换，工具路径为：

```bash
~/.mujoco/mujoco210/bin/compile
```

2. 使用转换工具转换为Mujoco XML，<font style="background-color:#FBDE28;">注意改变路径</font>

```bash
cd ~/.mujoco/mujoco210/bin/
./compile /home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/mujoco_model/ranger_mini.urdf /home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/mujoco_model/ranger_mini.xml
```

3. 转换成功后可以看到下面的文件

```xml
<mujoco model="ranger_mini_v3">
    <compiler angle="radian" meshdir="/home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/meshes/" />
    <size njmax="500" nconmax="100" />
    <asset>
        <mesh name="ranger_base" file="ranger_base.stl" scale="1000 1000 1000" />				<!-- 模型过大，需要改变比例 -->
        <mesh name="steering_wheel" file="steering_wheel.stl" scale="10 10 10" />
        <mesh name="wheel_v3" file="wheel_v3.stl" scale="10 10 10" />
    </asset>
    <worldbody>
        <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.792157 0.819608 0.933333 1" mesh="ranger_base" />
        <geom size="0.25 0.175 0.1" pos="0 0 -0.1" type="box" rgba="0.792157 0.819608 0.933333 1" />
        <body name="fr_steering_wheel_link" pos="0.25 -0.19 -0.1">
            <inertial pos="1.8493e-05 1.2713e-06 0.043679" quat="-0.070656 0.703532 0.070657 0.703603" mass="1" diaginertia="0.010465 0.00636302 0.00636248" />
            <joint name="fr_steering_joint" pos="0 0 0" axis="0 0 -1" limited="true" range="-2.1 2.1" />
            <geom type="mesh" contype="0" conaffinity="0" group="1" rgba="0.79216 0.81961 0.93333 1" mesh="steering_wheel" />
            <geom size="0.001 0.0005" type="cylinder" rgba="0.79216 0.81961 0.93333 1" />
            <body name="fr_wheel_link" pos="0 0 -0.12">
                <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="8" diaginertia="0.0324 0.02047 0.02047" />
                <joint name="fr_wheel" pos="0 0 0" axis="0 1 0" />
                <geom quat="0.000796327 1 0 0" type="mesh" contype="0" conaffinity="0" group="1" mesh="wheel_v3" />
                <geom size="0.09 0.04" pos="0 0 -0.005" quat="0.707388 0.706825 0 0" type="cylinder" />
            </body>
        </body>
        <body name="fl_steering_wheel_link" pos="0.25 0.19 -0.1">
            <inertial pos="-0.0017956 -9.6304e-08 0.043642" quat="-0.0165928 0.692206 0.0172886 0.721302" mass="1" diaginertia="0.0104649 0.006363 0.00636255" />
            <joint name="fl_steering_joint" pos="0 0 0" axis="0 0 -1" limited="true" range="-2.1 2.1" />
            <geom quat="0.000796327 0 0 1" type="mesh" contype="0" conaffinity="0" group="1" rgba="0.79216 0.81961 0.93333 1" mesh="steering_wheel" />
            <geom size="0.001 0.0005" type="cylinder" rgba="0.79216 0.81961 0.93333 1" />
            <body name="fl_wheel_link" pos="0 0 -0.12">
                <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="8" diaginertia="0.0324 0.02047 0.02047" />
                <joint name="fl_wheel" pos="0 0 0" axis="0 1 0" />
                <geom type="mesh" contype="0" conaffinity="0" group="1" mesh="wheel_v3" />
                <geom size="0.09 0.04" pos="0 0 -0.005" quat="0.707388 0.706825 0 0" type="cylinder" />
            </body>
        </body>
        <body name="rl_steering_wheel_link" pos="-0.25 0.19 -0.1">
            <inertial pos="-0.00017093 6.1255e-08 0.043678" quat="-0.020441 0.705442 0.0205219 0.708176" mass="1" diaginertia="0.0104651 0.006363 0.00636264" />
            <joint name="rl_steering_joint" pos="0 0 0" axis="0 0 -1" limited="true" range="-2.1 2.1" />
            <geom quat="0.000796327 0 0 1" type="mesh" contype="0" conaffinity="0" group="1" rgba="0.79216 0.81961 0.93333 1" mesh="steering_wheel" />
            <geom size="0.001 0.0005" type="cylinder" rgba="0.79216 0.81961 0.93333 1" />
            <body name="rl_wheel_link" pos="0 0 -0.12">
                <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="8" diaginertia="0.0324 0.02047 0.02047" />
                <joint name="rl_wheel" pos="0 0 0" axis="0 1 0" />
                <geom type="mesh" contype="0" conaffinity="0" group="1" mesh="wheel_v3" />
                <geom size="0.09 0.04" pos="0 0 -0.005" quat="0.707388 0.706825 0 0" type="cylinder" />
            </body>
        </body>
        <body name="rr_steering_wheel_link" pos="-0.23 -0.206 -0.1">
            <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01" />
            <joint name="rr_steering_joint" pos="0 0 0" axis="0 0 -1" limited="true" range="-2.1 2.1" />
            <geom type="mesh" contype="0" conaffinity="0" group="1" mesh="steering_wheel" />
            <geom size="0.001 0.0005" type="cylinder" />
            <body name="rr_wheel_link" pos="0 0 -0.12">
                <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="8" diaginertia="0.0324 0.02047 0.02047" />
                <joint name="rr_wheel" pos="0 0 0" axis="0 1 0" />
                <geom quat="0.000796327 1 0 0" type="mesh" contype="0" conaffinity="0" group="1" mesh="wheel_v3" />
                <geom size="0.09 0.04" pos="0 0 -0.005" quat="0.707388 0.706825 0 0" type="cylinder" />
            </body>
        </body>
    </worldbody>
</mujoco>

```

4. 检查转换是否正确，如果转换成功可以看到Mujoco仿真界面

```xml
cd ~/.mujoco/mujoco210/bin/
./simulate /home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/mujoco_model/ranger_mini.xml
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754560077045-b7b86938-f0f0-4646-b5a0-5dada2e87b24.png)

5. 转换完发现ranger mini的底盘模型过大，需要改变模型比例为`1 1 1`

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754560348982-acc52d11-e23f-42ab-aea5-54ea475afa61.png)

6. 修改完成后然后再次启动仿真工具，可以发现比例正常

```xml
cd ~/.mujoco/mujoco210/bin/
./simulate /home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/mujoco_model/ranger_mini.xml
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754560440128-f2d58d96-10a7-4247-82f1-75c084f90bfb.png)

## 完善Mujoco XML以仿真
1. 此时的Mujoco XML只能查看模型，我们无法控制它，所以需要为XML添加动作执行器和传感器，为仿真世界添加重力，风力等各种功能，Mujoco的XML标签文档是[https://mujoco.readthedocs.io/en/stable/XMLreference.html](https://mujoco.readthedocs.io/en/stable/XMLreference.html)，有特殊需求可以按照文档指示添加标签优化拟真程度
2. 下面展示完善后的XML文件

```xml
<mujoco model="ranger_mini_v3">
    <compiler angle="radian" meshdir="/home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/meshes/" />
    <size njmax="500" nconmax="100" />

    <!-- 全局物理参数 -->
    <option timestep="0.002"/>
    <option cone="elliptic"/>
    <option impratio="0.5"/>
    <option integrator="RK4"/>  <!-- 更稳定的积分器 -->
    <!-- Add air resistance -->
    <option wind="0 0 0" density="1.2" viscosity="1.8e-5"/>
    <!-- Enable gravity compensation -->
    <option gravity="0 0 -9.81"/>
    <option impratio="0.1"/>

    <asset>
        <mesh name="ranger_base" file="ranger_base.stl" scale="1 1 1" />
        <mesh name="steering_wheel" file="steering_wheel.stl" scale="10 10 10" />
        <mesh name="wheel_v3" file="wheel_v3.stl" scale="10 10 10" />
    </asset>
    <worldbody>
        <!-- 地面 (添加摩擦参数) -->
        <geom name="ground" type="plane" size="10 10 0.1" rgba="0.1 0.1 0.1 1" pos="0 0 -0.31"
        solimp="0.9 0.99 0.001" solref="0.01 1" friction="0.7 0.6 0.7" solmix="0.95"/>
        
        <body name="base_link" pos="0 0 0">
            <freejoint name="base_free_joint"/>
            <geom type="mesh" contype="0" conaffinity="0" group="1" mass="3" rgba="0.792157 0.819608 0.933333 1" mesh="ranger_base" friction="0.8 0.3 1"/>
            <!-- <geom size="0.25 0.175 0.1" pos="0 0 -0.1" type="box" rgba="0.792157 0.819608 0.933333 1" /> -->
            <body name="fr_steering_wheel_link" pos="0.25 -0.19 -0.1">
                <inertial pos="1.8493e-05 1.2713e-06 0.043679" quat="-0.070656 0.703532 0.070657 0.703603" mass="1" diaginertia="0.010465 0.00636302 0.00636248" />
                <joint name="fr_steering_joint" type="hinge" pos="0 0 0" axis="0 0 -1" limited="true" range="-2.1 2.1" damping="0.5"/>
                <geom type="mesh" contype="1" conaffinity="1" group="1" rgba="0.79216 0.81961 0.93333 1" mesh="steering_wheel" mass="0.2" friction="0.03 0.7 1" solimp="0.9 0.95 0.001" solref="0.01 1"/>
                <!-- <geom size="0.001 0.0005" type="cylinder" rgba="0.79216 0.81961 0.93333 1" /> -->
                <body name="fr_wheel_link" pos="0 0 -0.12">
                    <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="8" diaginertia="0.0324 0.02047 0.02047" />
                    <joint name="fr_wheel" type="hinge" pos="0 0 0" axis="0 1 0" />
                    <geom quat="0.000796327 1 0 0" type="mesh" contype="1" conaffinity="1" group="1" mesh="wheel_v3" friction="0.03 0.7 1" solimp="0.9 0.95 0.001" solref="0.01 1" mass="1"/>
                    <!-- <geom size="0.09 0.04" pos="0 0 -0.005" quat="0.707388 0.706825 0 0" type="cylinder" /> -->
                </body>
            </body>
            <body name="fl_steering_wheel_link" pos="0.25 0.19 -0.1">
                <inertial pos="-0.0017956 -9.6304e-08 0.043642" quat="-0.0165928 0.692206 0.0172886 0.721302" mass="1" diaginertia="0.0104649 0.006363 0.00636255" />
                <joint name="fl_steering_joint" type="hinge" pos="0 0 0" axis="0 0 -1" limited="true" range="-2.1 2.1" damping="0.5"/>
                <geom quat="0.000796327 0 0 1" type="mesh" contype="1" conaffinity="1" group="1" rgba="0.79216 0.81961 0.93333 1" mesh="steering_wheel" mass="0.2" friction="0.03 0.7 1" solimp="0.9 0.95 0.001" solref="0.01 1"/>
                <!-- <geom size="0.001 0.0005" type="cylinder" rgba="0.79216 0.81961 0.93333 1" /> -->
                <body name="fl_wheel_link" pos="0 0 -0.12">
                    <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="8" diaginertia="0.0324 0.02047 0.02047" />
                    <joint name="fl_wheel" type="hinge" pos="0 0 0" axis="0 1 0" />
                    <geom type="mesh" contype="1" conaffinity="1" group="1" mesh="wheel_v3" friction="0.03 0.7 1" solimp="0.9 0.95 0.001" solref="0.01 1" mass="1"/>
                    <!-- <geom size="0.09 0.04" pos="0 0 -0.005" quat="0.707388 0.706825 0 0" type="cylinder" /> -->
                </body>
            </body>
            <body name="rl_steering_wheel_link" pos="-0.25 0.19 -0.1">
                <inertial pos="-0.00017093 6.1255e-08 0.043678" quat="-0.020441 0.705442 0.0205219 0.708176" mass="1" diaginertia="0.0104651 0.006363 0.00636264" />
                <joint name="rl_steering_joint" type="hinge" pos="0 0 0" axis="0 0 -1" limited="true" range="-2.1 2.1" damping="0.5"/>
                <geom quat="0.000796327 0 0 1" type="mesh" contype="1" conaffinity="1" group="1" rgba="0.79216 0.81961 0.93333 1" mesh="steering_wheel" mass="0.2" friction="0.03 0.7 1" solimp="0.9 0.95 0.001" solref="0.01 1"/>
                <!-- <geom size="0.001 0.0005" type="cylinder" rgba="0.79216 0.81961 0.93333 1" /> -->
                <body name="rl_wheel_link" pos="0 0 -0.12">
                    <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="8" diaginertia="0.0324 0.02047 0.02047" />
                    <joint name="rl_wheel" type="hinge" pos="0 0 0" axis="0 1 0" />
                    <geom type="mesh" contype="1" conaffinity="1" group="1" mesh="wheel_v3" friction="0.03 0.7 1" solimp="0.9 0.95 0.001" solref="0.01 1" mass="1"/>
                    <!-- <geom size="0.09 0.04" pos="0 0 -0.005" quat="0.707388 0.706825 0 0" type="cylinder" /> -->
                </body>
            </body>
            <body name="rr_steering_wheel_link" pos="-0.23 -0.206 -0.1">
                <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01" />
                <joint name="rr_steering_joint" type="hinge" pos="0 0 0" axis="0 0 -1" limited="true" range="-2.1 2.1" damping="0.5"/>
                <geom type="mesh" contype="1" conaffinity="1" group="1" mesh="steering_wheel" mass="0.2" friction="0.03 0.7 1" solimp="0.9 0.95 0.001" solref="0.01 1"/>
                <!-- <geom size="0.001 0.0005" type="cylinder" /> -->
                <body name="rr_wheel_link" pos="0 0 -0.12">
                    <inertial pos="0 0 0" quat="0.5 0.5 -0.5 0.5" mass="8" diaginertia="0.0324 0.02047 0.02047" />
                    <joint name="rr_wheel" type="hinge" pos="0 0 0" axis="0 1 0" />
                    <geom quat="0.000796327 1 0 0" type="mesh" contype="1" conaffinity="1" group="1" mesh="wheel_v3" friction="0.03 0.7 1" solimp="0.9 0.95 0.001" solref="0.01 1" mass="1"/>
                    <!-- <geom size="0.09 0.04" pos="0 0 -0.005" quat="0.707388 0.706825 0 0" type="cylinder" /> -->
                </body>
            </body>
        </body>
    </worldbody>
    <actuator>
        <!-- 轮驱动 -->
        <motor name="fl_wheel_motor" joint="fl_wheel" gear="1" forcelimited="true" forcerange="-5 5"/>
        <motor name="fr_wheel_motor" joint="fr_wheel" gear="1" forcelimited="true" forcerange="-5 5"/>
        <motor name="rl_wheel_motor" joint="rl_wheel" gear="1" forcelimited="true" forcerange="-5 5"/>
        <motor name="rr_wheel_motor" joint="rr_wheel" gear="1" forcelimited="true" forcerange="-5 5"/>
        <!-- 转向驱动 -->
        <general name="fl_steering" joint="fl_steering_joint" 
                 gaintype="fixed" gainprm="10" biastype="none"/>
        <general name="fr_steering" joint="fr_steering_joint" 
                 gaintype="fixed" gainprm="10" biastype="none"/>
        <general name="rl_steering" joint="rl_steering_joint" 
                 gaintype="fixed" gainprm="10" biastype="none"/>        
        <general name="rr_steering" joint="rr_steering_joint" 
                 gaintype="fixed" gainprm="10" biastype="none"/>
    </actuator>
    <sensor>
        <!-- 转向关节位置反馈 -->
        <jointpos name="fl_steering_pos" joint="fl_steering_joint"/>
        <jointpos name="fr_steering_pos" joint="fr_steering_joint"/>
        <jointpos name="rl_steering_pos" joint="rl_steering_joint"/>
        <jointpos name="rr_steering_pos" joint="rr_steering_joint"/>
        
        <!-- 转向关节速度反馈 -->
        <jointvel name="fl_steering_vel" joint="fl_steering_joint"/>
        <jointvel name="fr_steering_vel" joint="fr_steering_joint"/>
        <jointvel name="rl_steering_vel" joint="rl_steering_joint"/>
        <jointvel name="rr_steering_vel" joint="rr_steering_joint"/>

        <!-- 轮速反馈 -->
        <jointvel name="fl_wheel_vel" joint="fl_wheel"/>
        <jointvel name="fr_wheel_vel" joint="fr_wheel"/>
        <jointvel name="rl_wheel_vel" joint="rl_wheel"/>
        <jointvel name="rr_wheel_vel" joint="rr_wheel"/>
    </sensor>

</mujoco>

```

3. 再次启动仿真程序，拖动Control面板的滑条控制小车移动

```xml
cd ~/.mujoco/mujoco210/bin/
./simulate /home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/mujoco_model/ranger_mini.xml
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1754560739406-b164c836-a57a-449c-9db3-7fb0d1548bae.png)

# 编写Mujoco中Ranger mini四轮四转底盘的PID控制器
## 完整程序代码
1. 在功能包中创建`scripts`文件夹，准备Python控制器文件

```xml
cd /home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3
mkdir scripts
cd scripts
touch ranger_mini_PID_controller.py
```

2. 准备PID控制器，其中包含对每个轮子单独控制的速度控制PID类、对每个舵单独控制的位置控制PID类以及对Ranger mini的简单运动学逆解

```python
#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
import math
import rospy
from geometry_msgs.msg import Twist

class VelocityPIDController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
        # PID parameters for velocity control (tuned values)
        self.kp = 40.0  # Proportional gain
        self.ki = 0.01   # Integral gain
        self.kd = 1    # Derivative gain
        
        # Initialize error terms for each wheel
        self.prev_error = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        self.integral = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        
        # Wheel joint and actuator names
        self.wheel_joints = {
            'fl': 'fl_wheel',
            'fr': 'fr_wheel',
            'rl': 'rl_wheel',
            'rr': 'rr_wheel'
        }
        
        self.wheel_actuators = {
            'fl': 'fl_wheel_motor',
            'fr': 'fr_wheel_motor',
            'rl': 'rl_wheel_motor',
            'rr': 'rr_wheel_motor'
        }
        
        # Control limits
        self.max_control = 10.0
        self.min_control = -10.0

    def compute_vel_pid(self, wheel, setpoint):
        """Compute velocity PID control for a single wheel."""
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.wheel_joints[wheel])
        current_vel = self.data.joint(joint_id).qvel[0]  # Using velocity instead of position
        
        error = setpoint - current_vel
        self.integral[wheel] += error
        derivative = error - self.prev_error[wheel]
        
        # Compute PID output
        output = (self.kp * error + 
                 self.ki * self.integral[wheel] + 
                 self.kd * derivative)
        
        # Apply output limits
        output = np.clip(output, self.min_control, self.max_control)
        self.prev_error[wheel] = error
        
        return output
    
    def set_wheel_vel(self, velocities):
        """Set target velocities for all wheels."""
        for wheel in ['fl', 'fr', 'rl', 'rr']:
            if wheel in velocities:
                control = self.compute_vel_pid(wheel, velocities[wheel])
                actuator_id = mujoco.mj_name2id(
                    self.model, 
                    mujoco.mjtObj.mjOBJ_ACTUATOR, 
                    self.wheel_actuators[wheel]
                )
                self.data.ctrl[actuator_id] = control

class AnglePIDController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
        # PID parameters for steering (tuned values)
        self.kp = 10.0  # Proportional gain
        self.ki = 0.01   # Integral gain
        self.kd = 1.0   # Derivative gain
        
        # Initialize error terms
        self.prev_error = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        self.integral = {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}
        
        # Steering joint and actuator names
        self.steering_joints = {
            'fl': 'fl_steering_joint',
            'fr': 'fr_steering_joint',
            'rl': 'rl_steering_joint',
            'rr': 'rr_steering_joint'
        }
        
        self.steering_actuators = {
            'fl': 'fl_steering',
            'fr': 'fr_steering',
            'rl': 'rl_steering',
            'rr': 'rr_steering'
        }
        
        # Control limits (in radians)
        self.max_control = math.pi/2   # 90 degrees
        self.min_control = -math.pi/2

    def compute_pos_pid(self, wheel, setpoint):
        """Compute position PID control for steering."""
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.steering_joints[wheel])
        current_pos = self.data.joint(joint_id).qpos[0]
        
        error = setpoint - current_pos
        self.integral[wheel] += error
        derivative = error - self.prev_error[wheel]
        
        # Compute PID output
        output = (self.kp * error + 
                 self.ki * self.integral[wheel] + 
                 self.kd * derivative)
        
        # Apply output limits
        output = np.clip(output, self.min_control, self.max_control)
        self.prev_error[wheel] = error
        
        return output
    
    def set_steering_angle(self, angles):
        """Set steering angles for all wheels."""
        for wheel in ['fl', 'fr', 'rl', 'rr']:
            if wheel in angles:
                control = self.compute_pos_pid(wheel, angles[wheel])
                actuator_id = mujoco.mj_name2id(
                    self.model, 
                    mujoco.mjtObj.mjOBJ_ACTUATOR, 
                    self.steering_actuators[wheel]
                )
                self.data.ctrl[actuator_id] = control

class FourWheelSteeringController:
    def __init__(self):
        # Robot dimensions (adjust according to your robot)
        self.track = 0.4  # Wheel track (distance between left and right wheels)
        self.wheel_steering_y_offset = 0.05  # Steering joint offset from wheel center
        self.wheel_radius = 0.1  # Wheel radius (m)
        self.wheel_base = 0.5  # Wheelbase (distance between front and rear wheels)
        
        # Current command
        self.lin_x = 0.0
        self.lin_y = 0.0
        self.ang = 0.0
        
        # ROS subscriber
        rospy.init_node('four_wheel_steering_controller')
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
    
    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel topic."""
        self.lin_x = msg.linear.x
        self.lin_y = msg.linear.y
        self.ang = msg.angular.z
    
    def calculate_wheel_commands(self):
        """Calculate wheel velocities and steering angles based on cmd_vel."""
        
        # Initialize outputs
        wheel_velocities = {}
        steering_angles = {}

        # Pure rotation
        if (self.lin_x == 0 and self.lin_y == 0 and self.ang != 0):
            # Compute the steering angle (same for all wheels, but left/right have opposite signs)
            steering_angle = math.atan2(self.wheel_base, self.track)
            rotate_direction = self.ang/abs(self.ang)
            
            steering_angles = {
                'fl': steering_angle,         # Front left: turn left
                'fr': -steering_angle,        # Front right: turn right
                'rl': -steering_angle,         # Rear left: turn left
                'rr': steering_angle         # Rear right: turn right
            }
            
            # Compute the wheel speed (same magnitude, direction depends on ang)
            wheel_speed = math.sqrt((self.track * 0.5)**2 + (self.wheel_base * 0.5)**2) * abs(self.ang)
            
            wheel_velocities = {
                'fl': wheel_speed*rotate_direction,            # Front left: forward
                'fr': -wheel_speed*rotate_direction,            # Front right: forward
                'rl': wheel_speed*rotate_direction,            # Rear left: forward
                'rr': -wheel_speed*rotate_direction             # Rear right: forward
            }

        # Translation (modified to handle reverse motion)
        if ((self.lin_x != 0 or self.lin_y != 0) and self.ang == 0):
            # Compute steering angle (same for forward/reverse)
            angle = math.atan2(self.lin_y, abs(self.lin_x))  # Use abs(lin_x) to avoid ±π
            
            # Compute speed magnitude
            speed = math.sqrt(math.pow(self.lin_y, 2) + math.pow(self.lin_x, 2))
            
            # Reverse speed if lin_x < 0
            if self.lin_x < 0:
                speed = -speed

            steering_angles = {
                'fl': angle,
                'fr': angle,
                'rl': angle,
                'rr': angle
            }
            wheel_velocities = {
                'fl': speed,
                'fr': speed,
                'rl': speed,
                'rr': speed
            }
        return wheel_velocities, steering_angles

def simulate():
    # Load model and data
    model = mujoco.MjModel.from_xml_path("/home/khalillee/sim1_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/mujoco_model/ranger_mini.xml")
    data = mujoco.MjData(model)
    
    # Initialize controllers
    pos_pid = AnglePIDController(model, data)
    vel_pid = VelocityPIDController(model, data)
    steering_controller = FourWheelSteeringController()
    
    # Initialize viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    
    # Camera settings
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -20
    viewer.cam.distance = 5
    viewer.cam.lookat = np.array([0.0, 0.0, 0.0])
    
    # Simulation loop
    try:
        while viewer.is_running() and not rospy.is_shutdown():
            sim_start = time.time()
            
            # Calculate wheel velocities and angles using ROS cmd_vel
            wheel_velocities, steering_angles = steering_controller.calculate_wheel_commands()
            
            # Update controllers
            pos_pid.set_steering_angle(steering_angles)
            vel_pid.set_wheel_vel(wheel_velocities)
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Sync viewer
            viewer.sync()
            
            # Real-time simulation
            time_until_next_step = model.opt.timestep - (time.time() - sim_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()

if __name__ == "__main__":
    simulate()
```

## 启动Mujoco仿真节点与teleop-twist-keyboard键盘控制节点
1. 启动ROS master

```bash
source your_ws/devel/setup.bash
roscore
```

2. 启动Mujoco仿真节点

```bash
source your_ws/devel/setup.bash
rosrun ranger_mini_v3 ranger_mini_PID_controller.py
```

3. 启动键盘控制节点

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

4. 支持全向平移与原地自旋运动，暂不支持旋转➕平移复合运动

[此处为语雀卡片，点击链接查看](https://www.yuque.com/docs/231440707#eCFMP)

