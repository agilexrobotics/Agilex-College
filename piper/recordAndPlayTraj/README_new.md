# 机械臂连续轨迹录制与播放

## 摘要
本文提供两个Python脚本（`recordTrajectory_new` 和 `playTrajectory_new`），用于录制和播放Piper机械臂的连续运动轨迹。与点位录制不同，此系统能够记录机械臂运动过程中的时间间隔信息，实现更加流畅和自然的动作复现。

## 功能特点
- **连续轨迹录制** (`recordTrajectory_new.py`)：实时记录机械臂运动过程中的关节弧度、夹爪状态及时间间隔
- **精确轨迹回放** (`playTrajectory_new.py`)：按照录制时的时间关系复现机械臂运动轨迹
- **倍速控制**：支持调整播放速度（0.1-2倍速），灵活控制回放节奏
- **安全机制**：包含超时检测、急停恢复和关节限制保护，确保操作安全

## 环境配置
- **操作系统**：Ubuntu（推荐Ubuntu 18.04或更高版本）
- **Python环境**：Python 3.6或更高版本
- **安装CAN工具**

    ```bash
    sudo apt install can-utils ethtool
    ```

- **安装piper_sdk**：

    ```bash
    pip3 install piper_sdk
    ```

- **参考文档**：https://github.com/agilexrobotics/piper_sdk/blob/master/README(ZH).MD

## 使用方法

### 1. 录制连续轨迹
```bash
python3 recordTrajectory_new.py
```
- 运行后按照提示开启示教模式
- 按回车键开始录制，机械臂移动时会自动记录轨迹点
- 录制完成后程序自动保存数据到`trajectory.csv`文件
- 录制时间可通过`record_time`参数设置（0表示无限录制）

### 2. 播放连续轨迹
```bash
python3 playTrajectory_new.py
```
- 确保机械臂已退出示教模式
- 按回车开始播放，机械臂将按照录制时的时间关系复现运动轨迹
- 可通过参数调整播放速度、次数等（需修改脚本内参数）

## 参数说明（脚本内可调整）

### recordTrajectory_new.py / recordTrajectory_new_en.py
- `have_gripper`：是否启用夹爪（默认 `True`）
- `record_time`：最大录制时间，单位：秒（0表示无限长）
- `timeout`：示教模式检测超时时间，单位：秒

### playTrajectory_new.py / playTrajectory_new_en.py
- `have_gripper`：是否启用夹爪（默认 `True`）
- `play_times`：播放次数（0为无限循环）
- `play_interval`：播放间隔，单位：秒
- `move_spd_rate_ctrl`：运动速度百分比（建议10-100）
- `play_speed`：播放倍速（建议范围：0.1-2）
- `timeout`：CAN模式切换超时时间，单位：秒

## 详细步骤
1. 机械臂上电，将USB转CAN模块与电脑连接（确保只连接一个CAN模块）

2. 打开终端，激活CAN模块  

    `sudo ip link set can0 up type can bitrate 1000000`

3. 克隆远程代码仓库

    `git clone https://github.com/agilexrobotics/Agilex-College.git`

4. 切换至`recordAndPlayTraj`目录

    `cd Agilex-College/piper/recordAndPlayTraj/`

5. 运行录制程序  

    `python3 recordTrajectory_new.py`

6. 短按示教按钮进入示教模式

    ![](https://cdn.nlark.com/yuque/0/2025/png/51616906/1755248995720-91c32fef-8189-48a9-9a1d-698dede6c9b5.png)

7. 摆放好机械臂的初始位置，终端回车后，拖动机械臂即可进行轨迹的录制

    ![](https://cdn.nlark.com/yuque/0/2025/png/51616906/1752571795765-29a3bc22-5c15-47ec-9d03-09939eb74290.png)

8. 录制结束后，再次短按示教按钮退出示教模式

    ![](https://cdn.nlark.com/yuque/0/2025/png/51616906/1755249001510-aa36e773-4b75-4e7e-85b6-f48a2d4f2950.png)

9. 播放前须知：  
初次退出示教模式时，需要经过特定的初始化过程才能从示教模式切换到CAN模式，因此播放程序会自动执行复位操作，将2、3、5号关节回到安全位置（零点），防止机械臂在重力作用下突然下落造成损坏，在特殊情况下需要人工辅助2、3、5关节回到零点

10. 运行播放程序

    `python3 playTrajectory_new.py`

11. 使能成功后，终端回车即可播放轨迹

    ![](https://cdn.nlark.com/yuque/0/2025/png/51616906/1752572703619-b75279b1-b93e-41a0-930e-05af4088abd4.png)