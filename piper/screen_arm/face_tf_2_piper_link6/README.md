# face_tf_2_piper_link6

> 本文件中的 `/home/grubaxu` 是原实验路径。统一仓库请以
> `screen_arm_ws/README.md` 的相对路径命令为准。

`face_tf_2_piper_link6` 是放在 `src/advx` 下的 ROS 2 功能包，用于把人脸 TF 转成 Piper 机械臂末端目标 TF。

节点读取：

```text
base_link -> face
```

节点发布：

```text
base_link -> arm_pose
```

`arm_pose` 的位置由 `face` 的位置和偏移计算，姿态由配置 RPY 固定。已有的 `piper_tf_control` 可以继续读取 `base_link -> arm_pose` 并发布 Piper 的 `PosCmd`。

## 计算方式

节点默认启动时先读取当前的 `base_link -> face` 和 `base_link -> link6`，多次采样求平均，自动标定 `base_link` 坐标系下从 `face` 到 `arm_pose` 的 xyz 偏移。标定完成后计算：

```text
arm_xyz = face_xyz_in_base + offset_xyz_in_base
```

如果自动标定超时，才会使用配置文件里的 `offset_xyz` 作为 fallback。

`arm_pose` 的姿态固定为配置中的：

```text
target_rpy: [0.306, 1.403, 0.280]
```

单位为 rad，对应规格中的 `base_link -> link6` 姿态。

## 构建

在工作空间根目录执行：

```bash
cd /home/grubaxu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select face_tf_2_piper_link6
source install/setup.bash
```

## 启动

```bash
ros2 launch face_tf_2_piper_link6 face_tf_2_piper_link6.launch.py
```

如果要和机械臂跟随节点一起使用，另开终端启动：

```bash
ros2 launch piper_tf_control piper_tf_control.launch.py
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `base_frame` | `base_link` | 输出 TF 的父坐标系，也是查找 face 的参考坐标系 |
| `face_frame` | `face` | 输入人脸 TF |
| `target_frame` | `arm_pose` | 输出机械臂目标 TF |
| `publish_rate_hz` | `30.0` | 发布频率，限制在 1 到 100 Hz |
| `offset_xyz` | `[-0.189, -0.044, -0.478]` | 自动标定失败时使用的 `base_link` 坐标系 fallback 偏移，单位 m |
| `target_rpy` | `[0.306, 1.403, 0.280]` | `arm_pose` 固定 RPY，单位 rad |
| `calibrate_from_link6` | `true` | 启动时是否用当前 `link6` 自动标定 xyz 偏移 |
| `calibration_frame` | `link6` | 自动标定使用的目标帧 |
| `calibration_sample_count` | `40` | 自动标定采样次数 |
| `calibration_timeout_sec` | `8.0` | 自动标定超时时间 |

配置文件：

```text
/home/grubaxu/ros2_ws/src/advx/face_tf_2_piper_link6/config/face_tf_2_piper_link6.yaml
```

## 单独链路测试

先虚拟发布规格中的 `base_link -> face`：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.721 --y -0.057 --z 0.438 \
  --qx 0.685 --qy 0.251 --qz 0.495 --qw 0.473 \
  --frame-id base_link --child-frame-id face
```

再启动本节点：

```bash
ros2 launch face_tf_2_piper_link6 face_tf_2_piper_link6.launch.py
```

查看输出：

```bash
ros2 run tf2_ros tf2_echo base_link arm_pose
```

单独测试时如果没有 `link6`，可以关闭自动标定，仅验证 fallback 偏移：

```bash
ros2 launch face_tf_2_piper_link6 face_tf_2_piper_link6.launch.py \
  calibrate_from_link6:=false
```

实际使用时保持默认自动标定。启动节点前，把机械臂当前 `link6` 放在你希望的 `arm_pose` 位置；节点会先采样当前 `face` 和 `link6`，标定完成后再发布 `arm_pose`。

本机实测时，当前 `base_link -> face` 和 `base_link -> link6` 采样 60 次得到：

```text
offset_xyz_in_base ~= [-0.522711, 0.009250, -0.124325]
```

`arm_pose` 的 RPY 仍为配置固定值 `[0.306, 1.403, 0.280]`。

测试期间不要同时运行其他发布 `arm_pose` 的节点，否则同一个 child frame 会有多个发布源。
