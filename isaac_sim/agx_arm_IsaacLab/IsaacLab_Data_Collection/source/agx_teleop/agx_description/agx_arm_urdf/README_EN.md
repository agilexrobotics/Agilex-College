# AgileX Robotic Arm URDF Models

[中文](./README.md)

This repository contains URDF / Xacro model files and 3D mesh resources for AgileX series robotic arms, for ROS / ROS2 visualization, simulation, and motion planning.

> **Scope**: This repository **primarily serves** the [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) main workspace (as a submodule, installed with the `agx_arm_description` package there).  
> If you use it outside the AgileX main repo, follow **Standalone use** below to create a **same-named** ROS package.

---

## Supported Models

| Model | Directory | Base URDF | Gripper Xacro | Dexterous Hand Xacro |
|-------|-----------|-----------|---------------|----------------------|
| Piper | `piper/` | `piper_description.urdf` | `piper_with_gripper_description.xacro` | `piper_with_left_revo2_description.xacro` / `piper_with_right_revo2_description.xacro` |
| Piper H | `piper_h/` | `piper_h_description.urdf` | `piper_h_with_gripper_description.xacro` | `piper_h_with_left_revo2_description.xacro` / `piper_h_with_right_revo2_description.xacro` |
| Piper L | `piper_l/` | `piper_l_description.urdf` | `piper_l_with_gripper_description.xacro` | `piper_l_with_left_revo2_description.xacro` / `piper_l_with_right_revo2_description.xacro` |
| Piper X | `piper_x/` | `piper_x_description.urdf` | `piper_x_with_gripper_description.xacro` | `piper_x_with_left_revo2_description.xacro` / `piper_x_with_right_revo2_description.xacro` |
| Nero | `nero/` | `nero_description.urdf` | `nero_with_gripper_description.xacro` | `nero_with_left_revo2_description.xacro` / `nero_with_right_revo2_description.xacro` |
| Revo2 Hand | `revo2/` | `revo2_left_hand.urdf` / `revo2_right_hand.urdf` | — | — |

---

## Directory Structure

```
agx_arm_urdf/
├── piper/
│   ├── meshes/dae/    # 3D mesh files (.dae)
│   └── urdf/          # URDF / Xacro files
├── piper_h/
│   ├── meshes/dae/
│   └── urdf/
├── piper_l/
│   ├── meshes/dae/
│   └── urdf/
├── piper_x/
│   ├── meshes/dae/
│   └── urdf/
├── nero/
│   ├── meshes/dae/
│   └── urdf/
└── revo2/
    ├── meshes/dae/
    └── urdf/
```

---

## Usage

### Recommended: use with the main repository

Clone [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) with submodules:

```bash
git clone -b ros2 --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git
```

Visualize the model in ROS2 (launch files are provided in the main repo):

```bash
ros2 launch agx_arm_description display.launch.py arm_type:=piper
```

For more details, see the [agx_arm_ros documentation](https://github.com/agilexrobotics/agx_arm_ros).

---

### Standalone use (your own workspace)

If you do not use the full `agx_arm_ros` workspace, you may still clone only this repository, but you must provide your own **ROS package**, and the package **name** must be: **`agx_arm_description`**

#### ROS 2 (ament_cmake)

```bash
mkdir -p ~/ws/src && cd ~/ws/src
ros2 pkg create --build-type ament_cmake agx_arm_description
cd agx_arm_description
git clone https://github.com/agilexrobotics/agx_arm_urdf.git agx_arm_urdf
```

In the package `CMakeLists.txt`:

```cmake
install(DIRECTORY agx_arm_urdf
  DESTINATION share/${PROJECT_NAME}
)
```

Then:

```bash
cd ~/ws
colcon build --packages-select agx_arm_description
source install/setup.bash
```

#### ROS 1 (catkin)

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
catkin_create_pkg agx_arm_description
cd agx_arm_description
git clone https://github.com/agilexrobotics/agx_arm_urdf.git agx_arm_urdf
```

In the package `CMakeLists.txt`:

```cmake
install(DIRECTORY agx_arm_urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

Then:

```bash
cd ~/catkin_ws
catkin_make   # or catkin build
source devel/setup.bash
```

---

## License

This project is released under the [MIT License](./LICENSE).
