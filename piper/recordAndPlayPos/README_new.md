# 机械臂固定点位录制与播放

## 摘要
本文提供两个Python脚本（`recordPos_new` 和 `playPos_new`），用于录制和播放Piper机械臂的固定点位。通过简单的操作，用户可以记录机械臂的关节位置及夹爪状态，并在需要时播放这些动作。

## 功能特点
- **录制功能** (`recordPos_new.py`)：实时记录机械臂各关节弧度及夹爪开合状态。
- **播放功能** (`playPos_new.py`)：读取记录的点位文件，控制机械臂复现动作。
- **安全机制**：包含超时检测、急停恢复和关节限制保护，确保操作安全。
- **灵活参数配置**：支持设置播放次数、间隔时间、运动速度等参数。

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

### 1. 录制固定点位
```bash
python3 recordPos_new.py
```
- 运行后按照提示开启示教模式。
- 按回车键记录当前位置，输入 `q` 退出录制。
- 点位将保存至同目录下的 `pos.csv` 文件。

### 2. 播放固定点位
```bash
python3 playPos_new.py
```
- 确保机械臂已退出示教模式。
- 按回车开始播放，机械臂将按记录顺序运动。
- 可通过参数调整播放次数、间隔时间等（需修改脚本内参数）。

## 参数说明（脚本内可调整）
### recordPos_new.py / recordPos_new_en.py
- `have_gripper`：是否启用夹爪（默认 `True`）
- `timeout`：示教模式检测超时时间（秒）

### playPos_new.py / playPos_new_en.py
- `have_gripper`：是否启用夹爪（默认 `True`）
- `play_times`：播放次数（0为无限循环）
- `play_interval`：点位间隔时间（负值需手动确认）
- `move_spd_rate_ctrl`：运动速度百分比（建议10-100）
- `timeout`：CAN模式切换超时时间（秒）

## 详细步骤
1. 机械臂上电，将USB转CAN模块与电脑连接（确保只连接一个CAN模块）

2. 打开终端，激活CAN模块

    `sudo ip link set can0 up type can bitrate 1000000`

3. 克隆远程代码仓库

    `git clone https://github.com/agilexrobotics/Agilex-College.git`

4. 切换至`recordAndPlayPos`目录

    `cd Agilex-College/piper/recordAndPlayPos/`

5. 运行录制程序

    `python3 recordPos_new.py`

6. 短按示教按钮进入示教模式

    ![](https://cdn.nlark.com/yuque/0/2025/png/51616906/1755248955456-fd0ddf6b-ef48-4f04-a13b-4352c8ab955a.png)

7. 摆放好机械臂的位置，终端回车进行点位的录制，输入'q'可结束录制  
    ![](https://cdn.nlark.com/yuque/0/2025/png/51616906/1752636233531-3e033cff-4df6-4b0c-81e8-0a2a85bddefc.png)

8. 录制结束后，再次短按示教按钮退出示教模式

    ![](https://cdn.nlark.com/yuque/0/2025/png/51616906/1755248964459-86f52f95-f5e1-49d2-8f0a-9c556b1b795b.png)

9. 播放前须知：初次退出示教模式时，需要经过特定的初始化过程才能从示教模式切换到CAN模式，因此播放程序会自动执行复位操作，将2、3、5号关节回到安全位置（零点），防止机械臂在重力作用下突然下落造成损坏，在特殊情况下需要人工辅助2、3、5关节回到零点

10. 运行播放程序

    `python3 playPos_new.py`

11. 使能成功后，终端回车即可播放点位

    ![](https://cdn.nlark.com/yuque/0/2025/png/51616906/1752636446142-96b80428-877d-43f3-9f33-ae6eacdbaab7.png)
