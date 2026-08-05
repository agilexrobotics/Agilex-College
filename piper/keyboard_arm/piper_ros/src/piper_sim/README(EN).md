# piper_sim

[中文](README.md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 Gazebo Simulation

### 0 Environment Setup

```bash
sudo apt update
sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
```

### 1.1 Piper Gazebo Simulation (With Gripper)

Run the Gazebo simulation:

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_gazebo piper_gazebo.launch.py
```

### 1.2 Piper Gazebo Simulation (Without Gripper)

```bash
ros2 launch piper_gazebo piper_no_gripper_gazebo.launch.py
```

**Note:** If controlling via MoveIt, you must start Gazebo first, then MoveIt. Also, use `piper_moveit.launch.py` instead of `demo.launch.py`.

## 2 Mujoco Simulation

### 2.1 Installing Mujoco 2.1.0 and mujoco-py

#### 2.1.1 Install Mujoco

1. [Download Mujoco 2.1.0](https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz)

2. Extract the files:

    ```bash
    mkdir ~/.mujoco
    cd (directory where the tar file is located)
    tar -zxvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco
    ```

3. Add environment variables:

    ```bash
    echo "export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:\$LD_LIBRARY_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Test the installation:

    ```bash
    cd ~/.mujoco/mujoco210/bin
    ./simulate ../model/humanoid.xml
    ```

#### 2.1.2 Install mujoco-py

1. Clone the source code:

    ```bash
    git clone https://github.com/openai/mujoco-py.git
    ```

2. Install (this step can be performed in a conda environment):

    ```bash
    cd ~/mujoco-py
    pip3 install -U 'mujoco-py<2.2,>=2.1'
    pip3 install -r requirements.txt
    pip3 install -r requirements.dev.txt
    python3 setup.py install
    sudo apt install libosmesa6-dev
    sudo apt install patchelf
    ```

3. Add environment variables:

    ```bash
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Test the installation:

    **Note:** If you encounter an error when importing `mujoco_py`, update `numpy`, `cmake`, or any other dependencies as instructed by the error message.

    Run the following Python script:

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

### 2.2 Piper Mujoco Simulation (With Gripper)

Run the Mujoco simulation:

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_mujoco_ctrl.py
```

**Note:** If you are unable to control the Mujoco robot arm, restart Mujoco.

To control the robot arm with the `rviz_gui` (run in a new terminal):

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_urdf.launch.py
```

### 2.3 Piper Mujoco Simulation (Without Gripper)

Run the Mujoco simulation:

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 run piper_mujoco piper_no_gripper_mujoco_ctrl.py
```

To control the robot arm with `rviz_gui` (run in a new terminal):

```bash
cd piper_ros
source install/setup.bash
```

```bash
ros2 launch piper_description display_no_gripper_urdf.launch.py
```

**Note:** If you cannot control the robot arm, run Mujoco after launching `rviz_gui`.

### Control Parameter Introduction

[Control parameters for the gripper version](../piper_description/mujoco_model/piper_description.xml)

[Control parameters for the non-gripper version](../piper_description/mujoco_model/piper_no_gripper_description.xml)

- `damping="100"` → Adjust joint damping
- `kp="10000"` → Adjust joint control gain
- `forcerange="-100 100"` → Adjust joint control torque range
