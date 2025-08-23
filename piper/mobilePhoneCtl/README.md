
# 脑洞大开之用手机陀螺仪控制机械臂

## 摘要
本文实现了通过手机传感器数据（加速度计、陀螺仪、磁力计）控制机械臂的功能。数据通过 WebSocket 从手机实时传输到本地 Python 脚本，脚本经姿态解算后控制机械臂运动。

## 标签
手机传感器、姿态遥操、陀螺仪、姿态解算、EKF

## 代码仓库
github链接：[https://github.com/agilexrobotics/Agilex-College/tree/master/piper/mobilePhoneCtl](https://github.com/agilexrobotics/Agilex-College/tree/master/piper/mobilePhoneCtl)

## 功能演示



## 环境配置

- 操作系统：Ubuntu（推荐Ubuntu 18.04或更高版本）
- Python环境：Python 3.6或更高版本

- 克隆项目：

    ```bash
    git clone https://github.com/agilexrobotics/Agilex-College.git
    cd Agilex-College/piper/mobilePhoneCtl/
    ```

- 安装依赖库：

    ```bash
    pip install -r requirements.txt --upgrade
    ```

- 确保已正确安装并配置 `piper_sdk` 及其硬件依赖。

## 手机 App 安装

本项目推荐使用 [Sensor Stream IMU+](https://www.sensorstream.app/)（付费 App）进行手机端数据采集与推送。

- 前往官网或应用商店购买并安装 Sensor Stream IMU+。
- 该 App 支持 iOS 和 Android。

## App 使用方法

1. 打开 Sensor Stream IMU+ App。
2. 在“Set IP Address”处输入运行本脚本的电脑 IP 地址和端口（默认 5000），如 `192.168.1.100:5000`。
3. 选择要推送的传感器（Accelerometer、Gyroscope、Magnetometer）。
4. 设置合适的 update interval（如 20ms）。
5. 点击“Start Streaming”开始推送数据。

## Python 脚本使用

1. 连接机械臂并激活CAN模块。

    ```bash
    sudo ip link set can0 up type can bitrate 1000000
    ```

2. 运行本目录下的 `main.py`：

    ```bash
    python3 main.py
    ```

3. 脚本启动后会显示本机 IP 地址和端口，请在 App 中填写一致。
4. 当 App 开始推送数据后，脚本会自动进行姿态解算，并通过 `piper_sdk` 控制机械臂末端姿态。

## 数据传输与机械臂控制说明

- 手机端通过 WebSocket 实时推送三轴加速度、陀螺仪和磁力计数据到 Python 脚本。
- 脚本使用扩展卡尔曼滤波（EKF）算法进行姿态解算，获得欧拉角（roll, pitch, yaw）。
- 解算结果实时通过 `piper_sdk` 的 `EndPoseCtrl` 接口发送给机械臂，实现姿态控制。

## 注意事项

- 请确保手机和电脑处于同一局域网内，且防火墙允许 5000 端口通信。
- 机械臂运动前请确保安全，避免碰撞。
- 若需修改端口或初始位置，请编辑 [`main.py`](main.py) 中相关参数。