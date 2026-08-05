# Piper_Moveit2

[中文](README.md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

| PYTHON  | STATE |
|---------|-------|
| ![humble](https://img.shields.io/badge/ros-humble-blue.svg) | ![Pass](https://img.shields.io/badge/Pass-blue.svg) |

> Note: If you encounter any issues during installation and usage, refer to Section 5.

## 1 Install Moveit2

1) **Binary Installation** - [Reference link](https://moveit.ai/install-moveit2/binary/)

    ```bash
    sudo apt install ros-humble-moveit*
    ```

2) **Source Compilation** - [Reference link](https://moveit.ai/install-moveit2/source/)

## 2 Environment Setup

After installing Moveit2, install the necessary dependencies:

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

If your system locale is not set to English, configure it as follows:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 3 Moveit Control for Real Robot Arm

### 3.1 Start `piper_ros`

After configuring [piper_ros](../../README.MD#1-Installation-Method):

```bash
cd ~/piper_ros
source install/setup.bash
bash can_activate.sh can0 1000000
```

Start the control node:

```bash
ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
```

### 3.2 Moveit2 Control

Start Moveit2:

```bash
cd ~/piper_ros
conda deactivate  # Remove this line if Conda is not installed
source install/setup.bash
```

#### 3.2.1 Run without gripper

```bash
ros2 launch piper_no_gripper_moveit demo.launch.py
```

#### 3.2.2 Run with gripper

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

![piper_moveit](../../asserts/pictures/piper_moveit.png)

You can control the robot arm by dragging the arrow at the end effector.

After adjusting the position, click **Plan & Execute** under the **Motion Planning** panel to start motion planning and execution.

## 4 Moveit Control for Simulated Robot Arm

### 4.1 Gazebo

#### 4.1.1 Run Gazebo

See [piper_gazebo](../piper_sim/README(EN).md#1-gazebo-simulation)

#### 4.1.2 Moveit Control

```bash
cd ~/piper_ros
source install/setup.bash
```

> Note: The following launch files are **not** the same as `demo.launch.py` used for real robots. They must be run **after Gazebo**; otherwise, the robot model will not appear.

Run with gripper:

```bash
ros2 launch piper_with_gripper_moveit piper_moveit.launch.py
```

Run without gripper:

```bash
ros2 launch piper_no_gripper_moveit piper_moveit.launch.py
```

### 4.2 Mujoco

#### 4.2.1 Moveit Control (Start Moveit First)

Same as [3.2 Moveit2 Control](#32-moveit2-control)

#### 4.2.2 Run Mujoco

See [piper_mujoco](../piper_sim/README(EN).md#2-mujoco-simulation)

Note: **You can close it using Ctrl+C+\\**

## 5 Potential Issues

### 5.1 Error when launching Gazebo: URDF not loaded, causing end effector and base to overlap

1. Check whether `install` contains the `piper_description` folder with a `config` directory, and ensure it includes the configuration files from `src/piper/piper_description/config`.

2. If `install` is missing the `urdf` folder, check if `src/piper/piper_description/urdf/piper_description_gazebo.xacro` (line 644) has the correct paths. If the issue persists, change the paths to absolute paths.

### 5.2 Error when running `demo.launch.py`

**Error:** Expected a `double`, but received a `string`.  
**Solution:** Run the following command:

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

Alternatively, set `LC_NUMERIC=en_US.UTF-8` before launching Moveit:

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch piper_moveit_config demo.launch.py
```
