## **机械臂实现Eigen线代库解算正逆运动学**

## 摘要
本项目实现基于线性代数库Eigen实现松灵PIPER机械臂的正解，逆解的雅各比方法，自定义交互式标记`interactive_marker_utils`的实现。涉及运动学正解、运动学逆解的雅各比方法、RVIZ仿真、机械臂DH、交互式标记、松灵PIPER

## 代码仓库
github链接：[**https://github.com/agilexrobotics/Agilex-College.git**](https://github.com/agilexrobotics/Agilex-College.git)

# 使用前准备
## 硬件准备
+ AgileX robotics Piper机械臂

## 软件环境配置
1. PIPER机械臂驱动部署请参考：[https://github.com/agilexrobotics/piper_sdk/blob/1_0_0_beta/README(ZH).MD](https://github.com/agilexrobotics/piper_sdk/blob/1_0_0_beta/README(ZH).MD)
2. PIPER机械臂ROS控制节点部署参考：[https://github.com/agilexrobotics/piper_ros/blob/noetic/README.MD](https://github.com/agilexrobotics/piper_ros/blob/noetic/README.MD)
3. 安装Eigen线性代数库

```bash
sudo apt install libeigen3-dev
```

## 准备松灵PIPER的DH参数表以及关节限位
查阅松灵PIPER用户手册可以找到PIPER的改进DH参数表与关节限位：

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753784251698-89bd82ef-c2e2-4e98-b1ed-371a697f2e13.png)

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753784668745-70afd1e0-f734-41af-b7bd-1b3ebfa44538.png)

---

# 正向运动学计算FK
正向运动学FK的计算过程实际上是<font style="background-color:#FBDE28;">从每个关节的角度值</font>----计算---->><font style="background-color:#FBDE28;">机械臂某一关节在三维世界的位姿</font>，本文以机械臂最后一个旋转关节joint6为例

## 准备DH参数
1. 根据PIPER的DH参数表构建正向运动学计算程序，由1.3小结的松灵PIPER的改进DH参数表，可以得到

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753784251698-89bd82ef-c2e2-4e98-b1ed-371a697f2e13.png)

```c
// 改进DH参数 [alpha, a, d, theta_offset]
dh_params_ = {
    {0,         0,          0.123,      0},                     // Joint 1
    {-M_PI/2,   0,          0,          -172.22/180*M_PI},      // Joint 2 
    {0,         0.28503,    0,          -102.78/180*M_PI},      // Joint 3
    {M_PI/2,    -0.021984,  0.25075,    0},                     // Joint 4
    {-M_PI/2,   0,          0,          0},                     // Joint 5
    {M_PI/2,    0,          0.091,      0}                      // Joint 6
};
```

转换为标准DH，可以参考以下的转换规则：

        * **<font style="color:rgb(64, 64, 64);">标准DH到改进DH</font>**<font style="color:rgb(64, 64, 64);">：</font>

<font style="color:rgb(64, 64, 64);">αᵢ₋₁(标准) = αᵢ(改进)</font>

<font style="color:rgb(64, 64, 64);">aᵢ₋₁(标准) = aᵢ(改进)</font>

<font style="color:rgb(64, 64, 64);">dᵢ(标准) = dᵢ(改进)</font>

<font style="color:rgb(64, 64, 64);">θᵢ(标准) = θᵢ(改进)</font>

        * **<font style="color:rgb(64, 64, 64);">改进DH到标准DH</font>**<font style="color:rgb(64, 64, 64);">：</font>

<font style="color:rgb(64, 64, 64);">αᵢ(标准) = αᵢ₊₁(改进)</font>

<font style="color:rgb(64, 64, 64);">aᵢ(标准) = aᵢ₊₁(改进)</font>

<font style="color:rgb(64, 64, 64);">dᵢ(标准) = dᵢ(改进)</font>

<font style="color:rgb(64, 64, 64);">θᵢ(标准) = θᵢ(改进)</font>

得到转换后的标准DH：

```c
// 标准DH参数 [alpha, a, d, theta_offset]
dh_params_ = {
    {-M_PI/2,   0,          0.123,      0},                     // Joint 1
    {0,         0.28503,    0,          -172.22/180*M_PI},      // Joint 2 
    {M_PI/2,    -0.021984,  0,          -102.78/180*M_PI},      // Joint 3
    {-M_PI/2,   0,          0.25075,    0},                     // Joint 4
    {M_PI/2,    0,          0,          0},                     // Joint 5
    {0,         0,          0.091,      0}                      // Joint 6
};
```

2. 准备DH变换矩阵
    - 改进DH变换矩阵：

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753866379644-a68c80dc-8d6c-4b0a-8877-287fd877f6e7.png)

    - 用Eigen改写为改进DH的变换矩阵：

```c
T << cos(theta),            -sin(theta),            0,             a,
     sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
     sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d,
     0,                     0,                      0,             1;
```

    - 标准DH变换矩阵：

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753866510048-392be307-9e24-4735-85d4-0e61ba2955c4.png)

    - 用Eigen改写为标准DH的变换矩阵：

```c
T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
     0,           sin(alpha),             cos(alpha),            d,
     0,           0,                      0,                     1;
```

3. 实现正向运动学运算的关键函数`computeFK()`，完整代码见代码仓库[**https://github.com/agilexrobotics/Agilex-College.git**](https://github.com/agilexrobotics/Agilex-College.git)

```cpp
Eigen::Matrix4d computeFK(const std::vector<double>& joint_values) {
    //检查输入关节值数量是否足够（至少6个）
    if (joint_values.size() < 6) {
        throw std::runtime_error("Piper arm requires at least 6 joint values for FK");
    }

    //初始化单位矩阵作为初始变换
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    //对每个关节：
    //    计算实际关节角度 = 输入值 + 偏移量
    //    获取固定参数d值
    //    计算当前关节的变换矩阵并累积到总变换
    for (size_t i = 0; i < 6; ++i) {
        double theta = joint_values[i] + dh_params_[i][3];  // θ = joint_value + θ_offset
        double d = dh_params_[i][2];                       // d = d_fixed (如果是旋转关节)

        T *= computeTransform(
            dh_params_[i][0],  // alpha
            dh_params_[i][1],  // a
            d,                 // d
            theta              // theta
            );
    }

    //返回最终变换矩阵
    return T;
}
```

## 验证FK计算准确性
1. 启动正向运动学验证程序

```bash
ros2 launch piper_kinematics test_fk.launch.py
```

2. 启动RVIZ仿真程序，开启显示TF树，观察计算的<font style="background-color:#FBDE28;">FK所得的机械臂末端</font>`link6_from_fk`的姿态是否和机器人原本的`link6`（由joint_state_publisher计算）是否重合

```bash
ros2 launch  piper_description display_piper_with_joint_state_pub_gui..launch.py 
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753868404196-18aa0d44-eeb5-4137-9977-b526d399b806.png)

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753868434120-16763e64-c292-491b-9ad5-813b38665b09.png)

可以看见重合度很高，且从`link6_from_fk`和`link6`的姿态可以看出误差基本在小数点后四位。

# 逆向运动学解算IK
逆向运动学IK的计算过程实际上是<font style="background-color:#FBDE28;">给定一个目标点</font>----计算---->><font style="background-color:#FBDE28;">机械臂每个关节要在什么位置才能使机械臂末端到达目标点</font>

## 确认关节限位
+ 需要确定PIPER机械臂每个关节的限位，确保IK解算出来的路径不会超过机械臂的限位，从而导致机械臂损坏或者其他危险情况
+ 由1.3节可以得知PIPER机械臂每个关节的限位为：

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753784668745-70afd1e0-f734-41af-b7bd-1b3ebfa44538.png)

+ 从而得到机械臂关节限位的矩阵

```cpp
std::vector<std::pair<double, double>> limits = {
    {-154/180*M_PI, 154/180*M_PI},    	// Joint 1
    {0, 			195/180*M_PI},      // Joint 2
    {-175/180*M_PI, 0},         		// Joint 3
    {-102/180*M_PI, 102/180*M_PI},      // Joint 4
    {-75/180*M_PI,  75/180*M_PI},       // Joint 5
    {-120/180*M_PI, 120/180*M_PI}       // Joint 6
};
```

## IK的雅各比矩阵方法实现的简要步骤
求解过程

1. **<font style="color:rgb(64, 64, 64);">计算误差e</font>**<font style="color:rgb(64, 64, 64);">：当前位姿与目标位姿的差异(6维向量：3位置+3姿态)</font>
2. **<font style="color:rgb(64, 64, 64);">误差e是否小于阈值？</font>**
    - **<font style="color:rgb(64, 64, 64);">是</font>**<font style="color:rgb(64, 64, 64);"> </font><font style="color:rgb(64, 64, 64);">→ 返回当前θ作为解</font>
    - **<font style="color:rgb(64, 64, 64);">否</font>**<font style="color:rgb(64, 64, 64);"> → 进入迭代优化步骤</font>
3. **<font style="color:rgb(64, 64, 64);">计算雅可比矩阵J</font>**<font style="color:rgb(64, 64, 64);">：6×6矩阵</font>
4. **<font style="color:rgb(64, 64, 64);">计算阻尼伪逆</font>**<font style="color:rgb(64, 64, 64);">：</font>

$ J⁺ = Jᵀ(JJᵀ + λ²I)⁻¹ $

<font style="color:rgb(64, 64, 64);">λ是阻尼系数，避免奇异位形时数值不稳定</font>

5. **<font style="color:rgb(64, 64, 64);">计算关节角度增量</font>**<font style="color:rgb(64, 64, 64);">：</font>

$ Δθ = J⁺e $

<font style="color:rgb(64, 64, 64);">通过误差e和伪逆计算关节角度的调整量</font>

6. **<font style="color:rgb(64, 64, 64);">更新关节角度</font>**<font style="color:rgb(64, 64, 64);">：</font>

$ θ = θ + Δθ $

<font style="color:rgb(64, 64, 64);">应用调整量到当前关节角度</font>

7. **<font style="color:rgb(64, 64, 64);">应用关节限制：</font>**
8. **<font style="color:rgb(64, 64, 64);">角度归一化</font>**
9. **<font style="color:rgb(64, 64, 64);">达到最大迭代?</font>**
    - **<font style="color:rgb(64, 64, 64);">否</font>**<font style="color:rgb(64, 64, 64);"> </font><font style="color:rgb(64, 64, 64);">→ 回到步骤2继续迭代</font>
    - **<font style="color:rgb(64, 64, 64);">是</font>**<font style="color:rgb(64, 64, 64);"> → 抛出未收敛错误</font>

关键函数`computeIK()`

```cpp
std::vector<double> computeIK(const std::vector<double>& initial_guess, 
                                 const Eigen::Matrix4d& target_pose,
                                 bool verbose = false,
                                 Eigen::VectorXd* final_error = nullptr) {
    //初始化一个猜测姿态（初始姿态）
    if (initial_guess.size() < 6) {
        throw std::runtime_error("Initial guess must have at least 6 joint values");
    }

    std::vector<double> joint_values = initial_guess;
    Eigen::Matrix4d current_pose;
    Eigen::VectorXd error(6);
    bool success = false;

    //开始迭代计算
    for (int iter = 0; iter < max_iterations_; ++iter) {
        //先计算初始状态的FK获取初始状态的位置与姿态
        current_pose = fk_.computeFK(joint_values);
        //计算初始状态的位置与姿态相较于目标点的误差
        error = computePoseError(current_pose, target_pose);

        if (verbose) {
            std::cout << "Iteration " << iter << ": error norm = " << error.norm() 
                      << " (pos: " << error.head<3>().norm() 
                      << ", orient: " << error.tail<3>().norm() << ")\n";
        }

        //检查误差是否小于阈值，分为位置误差阈值与姿态误差阈值
        if (error.head<3>().norm() < position_tolerance_ && 
            error.tail<3>().norm() < orientation_tolerance_) {
            success = true;
            break;
        }

        //计算雅各比矩阵（默认使用解析雅各比）
        Eigen::MatrixXd J = use_analytical_jacobian_ ? 
            computeAnalyticalJacobian(joint_values, current_pose) :
            computeNumericalJacobian(joint_values);

        //采用阻尼最小二乘法(Levenberg-Marquardt)
        //Δθ = Jᵀ(JJᵀ + λ²I)⁻¹e
        //θ_new = θ + Δθ
        Eigen::MatrixXd Jt = J.transpose();
        Eigen::MatrixXd JJt = J * Jt;
        //lambda_: 阻尼系数(默认0.1)，避免奇异位形时数值不稳定
        JJt.diagonal().array() += lambda_ * lambda_;
        Eigen::VectorXd delta_theta = Jt * JJt.ldlt().solve(error);

        //更新
        for (int i = 0; i < 6; ++i) {
            //应用调整量到当前关节角度
            double new_value = joint_values[i] + delta_theta(i);
            //确保更新后的θ在机械臂的物理限制范围内（关节角度限位）
            joint_values[i] = std::clamp(new_value, joint_limits_[i].first, joint_limits_[i].second);
        }

        //将关节角度规范到[-π,π]等标准范围内（避免不必要的多圈旋转）
        normalizeJointAngles(joint_values);
    }

    //如果超过最大迭代次数(100)还未求解出结果，抛出异常
    if (!success) {
        throw std::runtime_error("IK did not converge within maximum iterations");
    }

    //计算误差
    if (final_error != nullptr) {
        current_pose = fk_.computeFK(joint_values);
        *final_error = computePoseError(current_pose, target_pose);
    }

    return joint_values;
    }
```

## 使用interactive_marker以实现发布机械臂三维空间目标点
1. 安装ROS2依赖包

```cpp
sudo apt install ros-${ROS_DISTRO}-interactive-markers ros-${ROS_DISTRO}-tf2-ros
```

2. 启动interactive_marker_utils实现三维空间目标点发布

```cpp
ros2 launch interactive_marker_utils marker.launch.py 
```

3. 启动RVIZ2观察Marker

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753872321648-69a6f2ba-681b-4a26-81e2-9ac9a753f463.png)

4. 拖动Marker，并用ros2 topic echo 观察Marker发布的目标点是否有变化

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753872475114-5c107f79-12bd-4eed-a82d-4b3329bcc190.png)

## 在RVIZ中通过interactive_marker验证IK是否正确
1. 启动松灵PIPER的RVIZ仿真demo，因为此时没有`joint_state_publisher`，所以模型没有正确显示

```cpp
ros2 launch piper_description display_piper.launch.py 
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753872773158-ea4982f3-9280-4978-86ff-0452e233d76b.png)

2. 接下来启动IK节点和`interactive_marker`节点（在同一个launch文件里）,启动成功后可以看到机械臂正常显示

```cpp
ros2 launch piper_kinematics piper_ik.launch.py
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873508356-b36b3048-463d-428d-909b-3b55f47c3cc3.png)

3. 使用`interactive_marker`控制机械臂进行IK解算  
![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873573382-f4d0b195-f9f0-46f5-befb-ff6aca74e1c7.png)

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873620366-fbd691ad-50ea-4585-9730-98193a5565df.png)

4. 拖动`interactive_marker`可以看见IK成功解算出每个关节的角度

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873809425-e7d7617d-1284-4b2c-b66a-9a4bab7459de.png)

5. 如果拖动`interactive_marker`到达无法解算的地方则会抛出异常

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753873898378-0ad9897d-b22d-4093-845e-a91c0d730e92.png)

# 在真实PIPER机械臂上验证IK
1. 首先启动连接PIPER的CAN通信的脚本

```cpp
cd piper_ros
./find_all_can_port.sh 
./can_activate.sh 
```

![](https://cdn.nlark.com/yuque/0/2025/png/51431964/1753874234843-d776e6b7-0a07-4e3d-a93a-939388f15544.png)

2. 启动PIPER真机控制节点

```cpp
ros2 launch piper my_start_single_piper_rviz.launch.py 
```

3. 接下来启动IK节点和`interactive_marker`节点（在同一个launch文件里），可以看到机械臂运动到HOME点

```cpp
ros2 launch piper_kinematics piper_ik.launch.py
```

4. 拖动`interactive_marker`并观察PIPER机械臂运动情况

[此处为语雀卡片，点击链接查看](https://www.yuque.com/docs/230096875#FvOGu)





