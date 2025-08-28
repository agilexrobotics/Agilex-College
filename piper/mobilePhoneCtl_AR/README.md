# 手机AR遥操机械臂——脑洞大开的位姿控制新体验

## 摘要

本文通过手机AR技术实现直观的位姿控制。只需一部Android手机，您就可以在增强现实环境中远程操控PiPER机械臂，带来前所未有的交互体验。

## 标签
PiPER机械臂、AR遥操作、增强现实、位姿控制、夹爪控制

## 仓库

- **导航仓库**：[https://github.com/agilexrobotics/Agilex-College](https://github.com/agilexrobotics/Agilex-College)

- **项目仓库**：[https://github.com/kehuanjack/PiPER_AR_Teleop](https://github.com/kehuanjack/PiPER_AR_Teleop)

## 功能演示



## 环境配置
- 操作系统：Ubuntu（推荐Ubuntu 18.04或更高版本）
- Python环境：Python 3.10或更高版本
- 克隆项目并切换至项目根目录下：

    ```bash
    git clone https://github.com/kehuanjack/PiPER_AR_Teleop.git
    cd PiPER_AR_Teleop
    ```

- 安装Python依赖库：

    ```bash
    pip3 install -r requirements.txt --upgrade
    ```

- 安装手机客户端：
  [点击此处下载](https://github.com/suessmann/daxie/releases/download/apk/app-daxie.apk)

## 执行步骤

1. **连接机械臂并激活CAN模块**：`sudo ip link set can0 up type can bitrate 1000000`
2. **启用服务**：在项目目录下运行`python3 main.py`开启服务
2. **连接建立**：打开手机客户端，在设置中输入电脑IP地址和端口建立连接
3. **环境扫描**：缓慢移动手机扫描环境建立AR空间，直到出现网格平面
4. **定位点放置**：在合适的平面上点击屏幕放置虚拟定位点作为参考坐标系
5. **开始控制**：按下音量上键启动遥操作，移动手机即可控制机械臂
6. **夹爪控制**：按下音量下键切换夹爪开合状态

## 注意事项

- 请确保手机和电脑处于同一局域网内
- 机械臂运行期间请保持安全距离，切勿靠近运动中的机械臂
- 使用手机进行远程操作时请保持谨慎。AR姿态估计依赖于光照条件和图像质量，建议在光线良好的环境下操作