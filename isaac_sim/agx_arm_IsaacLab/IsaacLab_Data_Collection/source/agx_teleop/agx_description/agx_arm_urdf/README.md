# AgileX 机械臂 URDF 模型

[English](./README_EN.md)

本仓库包含 AgileX 系列机械臂的 URDF / Xacro 模型文件及对应的 3D 网格（mesh）资源，供 ROS / ROS2 可视化、仿真和运动规划使用。

> **定位说明**：本仓库**主要服务于** [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) 主仓库（作为其中的子模块，与主仓内的 `agx_arm_description` 功能包一同安装）。  
> 若你仅在非 AgileX 主仓场景下使用，可按下文「独立使用」自行创建**同名**功能包。

---

## 支持的型号

| 型号 | 目录 | 基础 URDF | 夹爪 Xacro | 灵巧手 Xacro |
|------|------|-----------|------------|--------------|
| Piper | `piper/` | `piper_description.urdf` | `piper_with_gripper_description.xacro` | `piper_with_left_revo2_description.xacro` / `piper_with_right_revo2_description.xacro` |
| Piper H | `piper_h/` | `piper_h_description.urdf` | `piper_h_with_gripper_description.xacro` | `piper_h_with_left_revo2_description.xacro` / `piper_h_with_right_revo2_description.xacro` |
| Piper L | `piper_l/` | `piper_l_description.urdf` | `piper_l_with_gripper_description.xacro` | `piper_l_with_left_revo2_description.xacro` / `piper_l_with_right_revo2_description.xacro` |
| Piper X | `piper_x/` | `piper_x_description.urdf` | `piper_x_with_gripper_description.xacro` | `piper_x_with_left_revo2_description.xacro` / `piper_x_with_right_revo2_description.xacro` |
| Nero | `nero/` | `nero_description.urdf` | `nero_with_gripper_description.xacro` | `nero_with_left_revo2_description.xacro` / `nero_with_right_revo2_description.xacro` |
| Revo2 灵巧手 | `revo2/` | `revo2_left_hand.urdf` / `revo2_right_hand.urdf` | — | — |

---

## 目录结构

```
agx_arm_urdf/
├── piper/
│   ├── meshes/dae/    # 3D 网格文件（.dae）
│   └── urdf/          # URDF / Xacro 文件
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

## 使用方式

### 推荐：随主仓库使用

通过 [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) 克隆（含子模块）：

```bash
git clone -b ros2 --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git
```

在 ROS2 中加载模型进行可视化（launch 由主仓提供）：

```bash
ros2 launch agx_arm_description display.launch.py arm_type:=piper
```

更多用法请参阅 [agx_arm_ros 文档](https://github.com/agilexrobotics/agx_arm_ros)。

---

### 独立使用（自建工作空间）

在不使用整个 `agx_arm_ros` 时，仍可只克隆本仓库，但需自己提供 **ROS 功能包**，且功能包**名称**必须为：`agx_arm_description`

#### ROS 2（ament_cmake）

```bash
mkdir -p ~/ws/src && cd ~/ws/src
ros2 pkg create --build-type ament_cmake agx_arm_description
cd agx_arm_description
git clone https://github.com/agilexrobotics/agx_arm_urdf.git agx_arm_urdf
```

在包内 `CMakeLists.txt` 中配置：

```cmake
install(DIRECTORY agx_arm_urdf
  DESTINATION share/${PROJECT_NAME}
)
```

然后：

```bash
cd ~/ws
colcon build --packages-select agx_arm_description
source install/setup.bash
```

#### ROS 1（catkin）

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
catkin_create_pkg agx_arm_description
cd agx_arm_description
git clone https://github.com/agilexrobotics/agx_arm_urdf.git agx_arm_urdf
```

在包内 `CMakeLists.txt` 中配置：

```cmake
install(DIRECTORY agx_arm_urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

然后：

```bash
cd ~/catkin_ws
catkin_make   # 或 catkin build
source devel/setup.bash
```

---

## 许可证

本项目基于 [MIT License](./LICENSE) 发布。
