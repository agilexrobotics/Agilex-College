# piper_pos_slider

> 本文件中的 `/home/grubaxu` 是原实验路径。统一仓库请以
> `screen_arm_ws/README.md` 的相对路径命令为准。

`piper_pos_slider` 是一个给 Piper 机械臂末端位姿发 `PosCmd` 的 ROS2 Python 功能包。

它的用途是替代手动执行下面这种一次性发布方式：

```bash
ros2 topic pub /pos_cmd piper_msgs/msg/PosCmd "{x: 0.25, y: 0.00, z: 0.27, roll: 0.00, pitch: 1.57, yaw: 0.00, gripper: 0.0, mode1: 0, mode2: 0}"
```

这个包提供两种运行方式：

1. 图形界面模式，提供 6 个拖动滑块，实时发布末端目标位姿。
2. 测试模式 `--test-z-motion`，只做 z 轴受限动作测试，便于安全验证。

## 依赖

- `rclpy`
- `geometry_msgs`
- `piper_msgs`
- 运行 GUI 模式时还需要系统自带 `tkinter`

## 构建

在工作空间根目录执行：

```bash
cd /home/grubaxu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select piper_pos_slider --symlink-install
source install/setup.bash
```

## 运行 GUI 滑块节点

```bash
ros2 run piper_pos_slider piper_pos_slider
```

GUI 中 6 个滑块含义如下：

- 滑块 1: `x`
- 滑块 2: `y`
- 滑块 3: `z`
- 滑块 4: `roll`
- 滑块 5: `yaw`
- 滑块 6: `pitch`

注意这里的滑块显示顺序按需求写成 `roll、yaw、pitch`，但发布到 `PosCmd` 时仍然会正确映射为：

- `roll`
- `pitch`
- `yaw`

节点默认以 20 Hz 发布到 `/pos_cmd`。

## 安全限制

测试时只允许控制 `z` 轴在 `0.21 ~ 0.41` 之间。

节点内部会对 `z` 做硬限幅，超过范围的值会被自动裁剪到安全区间。

## 测试模式

测试模式会先读取当前 `/end_pose` 作为基准，然后只改变命令中的 `z`，适合做安全范围内的上下小幅动作测试。

运行示例：

```bash
ros2 run piper_pos_slider piper_pos_slider --test-z-motion --test-z-values 0.230,0.250,0.230 --test-hold-sec 2.0
```

参数说明：

- `--test-z-values`：用逗号分隔的 z 目标值列表
- `--test-hold-sec`：每个目标保持发布的时间，单位秒

示例中的所有 z 值都在允许范围内。你也可以换成其他 `0.21 ~ 0.41` 内的值。

## 初始化位姿

节点默认使用下面这组初始化位姿：

```yaml
position:
  x: 0.055142
  y: -0.002581
  z: 0.217777
orientation:
  x: 0.0009725635585632747
  y: 0.6530434831441294
  z: -0.029481433351552566
  w: 0.7567457355880148
```

其中姿态输入是四元数，节点启动后会自动转换为欧拉角。

## 常用检查命令

查看节点是否在线：

```bash
ros2 node list
```

查看 `/pos_cmd` 是否有订阅者：

```bash
ros2 topic info /pos_cmd -v
```

查看末端反馈：

```bash
ros2 topic echo /end_pose
```

## 说明

这个包只负责发末端位姿命令，不包含机械臂驱动本身。
机械臂驱动节点需要先正常运行，并且机械臂已经上电、使能、连接完成。
