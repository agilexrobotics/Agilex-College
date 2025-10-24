# 手柄遥操机械臂——直观的机械臂控制新体验

## 摘要

本文通过游戏手柄实现直观的机械臂控制。使用标准的游戏手柄，您可以在可视化环境中操控PiPER机械臂，带来精准而直观的控制体验。

## 标签
PiPER机械臂、手柄遥操、关节控制、位姿控制、夹爪控制、运动学正逆解

## 仓库

- **导航仓库**: https://github.com/agilexrobotics/Agilex-College
- **项目仓库**: https://github.com/kehuanjack/Gamepad_PiPER

## 功能演示

[![](https://i.ytimg.com/an_webp/smTTbOfdTlk/mqdefault_6s.webp?du=3000&sqp=CJXS7McG&rs=AOn4CLBfiWjTsfz7kRpplE8f4Wx6WtDRlg)](https://youtu.be/smTTbOfdTlk)

## 环境配置
- 操作系统：Ubuntu 20.04或更高版本

- Python环境：Python 3.9或更高版本，推荐使用Anaconda或Miniconda

- 克隆项目并切换至项目根目录下：

   ```bash
   git clone https://github.com/kehuanjack/Gamepad_PiPER.git
   cd Gamepad_PiPER
   ```

- 安装通用的依赖库和运动学模块的依赖库（三选一，推荐使用cuRobo库）：

   - 基于[pinocchio](https://github.com/stack-of-tasks/pinocchio)库（Python == 3.9，需要安装[piper_ros](https://github.com/agilexrobotics/piper_ros)，并source机械臂的ros工作空间，否则会找不到meshes文件）：

      ```bash
      conda create -n test_pinocchio python=3.9.* -y
      conda activate test_pinocchio
      pip3 install -r requirements_common.txt --upgrade
      conda install pinocchio=3.6.0 -c conda-forge
      pip install meshcat
      pip install casadi
      ```

      需要在`main.py`和`main_virtual.py`文件中选择`from src.gamepad_pin import RoboticArmController`

   - 基于[PyRoKi](https://github.com/chungmin99/pyroki)库（Python >= 3.10）:

      ```bash
      conda create -n test_pyroki python=3.10.* -y
      conda activate test_pyroki
      pip3 install -r requirements_common.txt --upgrade
      pip3 install pyroki@git+https://github.com/chungmin99/pyroki.git@f234516
      ```

      需要在`main.py`和`main_virtual.py`文件中选择`from src.gamepad_limit import RoboticArmController`或`from src.gamepad_no_limit import RoboticArmController`

   - 基于[cuRobo](https://github.com/NVlabs/curobo)库（Python >= 3.8，推荐的CUDA版本为11.8）:

      ```bash
      conda create -n test_curobo python=3.10.* -y
      conda activate test_curobo
      pip3 install -r requirements_common.txt --upgrade
      sudo apt install git-lfs && cd ../
      git clone https://github.com/NVlabs/curobo.git && cd curobo
      pip3 install "numpy<2.0" "torch==2.0.0" pytest lark
      pip3 install -e . --no-build-isolation
      python3 -m pytest .
      cd ../Gamepad_PiPER
      ```

      需要在`main.py`和`main_virtual.py`文件中选择`from src.gamepad_curobo import RoboticArmController`

## 执行步骤

1. **连接机械臂并激活CAN模块**：`sudo ip link set can0 up type can bitrate 1000000`

2. **连接游戏手柄**：将手柄通过USB或蓝牙连接到电脑

3. **启用控制服务**：在项目目录下运行`python3 main.py`或`python3 main_virtual.py`，建议先运行`main_virtual.py`进行虚拟机械臂测试

4. **手柄连接验证**：程序启动后，检查控制台输出确认手柄已正确识别

5. **网页可视化**：打开浏览器，输入`http://localhost:8080`访问网页，可视化显示机械臂状态

6. **开始控制**：按照手柄映射说明操作机械臂

## 手柄控制说明

### 按钮功能映射

| 按钮 | 短按功能 | 长按功能 |
|------|----------|----------|
| **HOME** | 连接/断开机械臂 | 无 |
| **START** | 切换上层的控制模式（关节/位姿）| 切换底层的控制模式（关节/位姿）|
| **BACK** | 切换底层的命令模式（位置速度0x00/快速响应0xAD）| 无 |
| **Y** | 回零位置 | 无 |
| **A** | 保存当前位置 | 清除当前保存的位置 |
| **B** | 恢复上一个保存的位置 | 无 |
| **X** | 切换位置回放顺序 | 清除所有保存的位置 |
| **LB** | 增加速度因子（上层） | 减少速度因子（上层） |
| **RB** | 增加移动速度（底层） | 减少移动速度（底层） |

### 摇杆与扳机功能

| 控制元件 | 关节模式功能 | 位姿模式功能 |
|----------|--------------|--------------|
| **左摇杆** | J1（底座旋转）：左右<br/>J2（大臂）：上下 | 末端X/Y轴移动 |
| **右摇杆** | J3（小臂）：上下<br/>J6（腕部旋转）：左右 | 末端Z轴移动和绕Z轴旋转 |
| **方向键** | J4（腕部偏航）：左右<br/>J5（腕部俯仰）：上下 | 末端绕X/Y轴旋转 |
| **左扳机 (LT)** | 关闭夹爪 | 关闭夹爪 |
| **右扳机 (RT)** | 打开夹爪 | 打开夹爪 |

### 特殊功能说明

1. **夹爪控制**:
   - 夹爪开合程度范围: 0-100%
   - 特殊跳变功能: 当夹爪处于完全关闭（0%）或完全打开（100%）状态时，快速按下并释放扳机可实现状态跳变

2. **速度控制**:
   - 速度因子: 0.25x, 0.5x, 1.0x, 2.0x, 3.0x, 4.0x, 5.0x（通过LB切换）
   - 移动速度: 10%-100%（通过RB切换）

3. **位置记忆**:
   - 可保存多个位置点
   - 支持顺序和逆序回放

## 注意事项

- 可以先运行`main_virtual.py`进行虚拟机械臂测试
- 初次使用建议从低速模式开始，熟悉操作后再提高速度
- 机械臂运行期间请保持安全距离，切勿靠近运动中的机械臂，否则后果自负
- 数值解在接近临界点时可能出现大幅度的关节跳动，请保持安全距离，否则后果自负
- 快速响应模式（0xAD）很危险，请谨慎使用，如要使用请保持安全距离，否则后果自负
- 如果选择使用pinocchio库，需要提前source机械臂的ros工作空间，否则会找不到meshes文件