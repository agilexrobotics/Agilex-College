# piper_tf_control

> 本文件中的 `/home/grubaxu` 是原实验路径。统一仓库请以
> `screen_arm_ws/README.md` 的相对路径命令为准。

`piper_tf_control` 是一个 ROS 2 Python 功能包。节点以 30 Hz 持续读取
`base_link -> arm_pose` TF，并将 TF 中的完整末端位姿发布到 Piper 驱动的
`pos_cmd` 话题。

默认还启用了平滑模式。它不会让目标位姿一步到位，而是限制每个发布周期里的
最大平移和最大转角变化，因此比直通模式更稳。

## 节点行为

节点只负责完成下面的映射：

```text
TF: base_link -> arm_pose
                  |
                  v
Topic: /pos_cmd  piper_msgs/msg/PosCmd
```

每个控制周期都会读取：

- TF 平移：`x/y/z`
- TF 旋转：转换为 `roll/pitch/yaw`

节点不会：

- 在 TF 存在时读取 `/end_pose` 作为命令输入
- 使用反馈位姿替换 TF 位姿
- 对 x、y、z 做限位
- 对 roll、pitch、yaw 做限位
- 仅允许某一个轴运动

如果 `base_link -> arm_pose` 暂时不存在，节点会使用最新 `/end_pose` 反馈持续
发布保持当前位置的 `PosCmd`。如果启动后既没有 TF，也没有收到过 `/end_pose`，
节点会等待，不会发布零位姿。

`arm_pose` 通常由其他目标生成节点发布。只要 TF 树中存在正确的
`base_link -> arm_pose`，本节点就会跟随它的完整位置和姿态。

## 坐标系和单位

根据当前 Piper 驱动实现：

- `PosCmd.x/y/z`：米
- `PosCmd.roll/pitch/yaw`：弧度

驱动将这些值直接转换为 Piper 末端控制接口的坐标，因此本节点默认不再做
额外的坐标变换。

## 构建

```bash
cd /home/grubaxu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_tf_control --symlink-install
source install/setup.bash
```

## 启动

先启动 Piper 驱动并确认机械臂已经上电、使能：

```bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=true \
  gripper_exist:=false
```

再启动本节点：

```bash
ros2 launch piper_tf_control piper_tf_control.launch.py
```

也可以直接使用参数文件：

```bash
ros2 run piper_tf_control piper_tf_control \
  --ros-args --params-file \
  /home/grubaxu/ros2_ws/install/piper_tf_control/share/piper_tf_control/config/piper_tf_control.yaml
```

## TF 发布方约定

目标发布节点需要提供：

```text
parent frame: base_link
child frame:  arm_pose
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link arm_pose
```

检查命令：

```bash
ros2 topic info /pos_cmd -v
ros2 topic echo /pos_cmd
```

## 测试说明

节点本身不限制运动范围。为了符合当前测试要求，包内提供
`piper_tf_z_test` 动态 TF 测试发布器。它会读取当前 `/end_pose` 作为基准，
保持 x/y/姿态不变，只改变 `arm_pose` 的 z。

先启动控制节点：

```bash
ros2 launch piper_tf_control piper_tf_control.launch.py
```

再运行 z 轴测试：

```bash
ros2 run piper_tf_control piper_tf_z_test \
  --sequence 0.40:10.0,0.21:10.0
```

含义：

- 读取一次当前 `/end_pose`，保存当前 x/y/姿态。
- 发布 `base_link -> arm_pose`，z 设为 `0.40 m`，保持 10 秒。
- 继续发布同一个 TF，z 改为 `0.21 m`，保持 10 秒。

如果需要自定义测试序列：

```bash
ros2 run piper_tf_control piper_tf_z_test \
  --sequence 0.30:5.0,0.35:5.0,0.21:10.0
```

测试期间不要运行其他 `arm_pose` 发布节点，否则同一个 child frame 会出现多个
发布源。

观察实际反馈：

```bash
ros2 topic echo /end_pose
```

检查当前命令：

```bash
ros2 topic echo /pos_cmd
```

如果想验证 TF 缺失保持原地，可以停止 `piper_tf_z_test` 或其他 `arm_pose`
发布节点，然后继续观察 `/pos_cmd` 是否保持在当前 `/end_pose` 附近：

```bash
ros2 topic echo /end_pose
ros2 topic echo /pos_cmd
```

实际使用时，目标 TF 应由其他节点持续发布；本测试命令只用于验证控制链路。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `base_frame` | `base_link` | TF 父坐标系 |
| `target_frame` | `arm_pose` | TF 目标坐标系 |
| `command_topic` | `pos_cmd` | Piper `PosCmd` 发布话题 |
| `feedback_topic` | `end_pose` | TF 缺失时用于保持原地的末端反馈 |
| `publish_rate_hz` | `30.0` | 命令发布频率，内部限制为 1~100 Hz |
| `smoothing_enabled` | `true` | 是否启用速度限幅平滑 |
| `max_linear_speed_m_s` | `0.03` | 最大平移速度 |
| `max_angular_speed_rad_s` | `0.6` | 最大角速度 |
| `position_deadband_m` | `0.002` | 目标变化小于该距离时保持上一条命令 |
| `angle_deadband_rad` | `0.03` | 目标角度变化小于该值时保持上一条命令 |
| `gripper` | `0.0` | 夹爪命令 |
| `mode1` | `0` | Piper 模式字段 |
| `mode2` | `0` | Piper 模式字段 |

节点没有位置或姿态限位参数。机械臂的安全范围应由上游目标生成节点、
现场测试流程和机械臂自身控制系统共同保证。

如果你觉得还是快，可以继续把 `max_linear_speed_m_s` 调小，例如 `0.02`；
如果你想更贴近原始 TF，可以把它调大，或者直接把 `smoothing_enabled` 设为
`false`。

如果到位后还有轻微晃动，优先调大死区：

```yaml
position_deadband_m: 0.003
angle_deadband_rad: 0.04
```

死区的含义是：目标变化小于阈值时，节点继续保持上一条命令，不追这些微小
变化。默认值是 `2 mm` 和 `0.03 rad`。
