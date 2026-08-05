cmake_minimum_required(VERSION 3.5)
project(piper_mujoco)

# 设置 CMake 的编译选项
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# 创建 Python 脚本安装规则
install(PROGRAMS
  scripts/piper_mujoco_ctrl.py
  scripts/piper_no_gripper_mujoco_ctrl.py
  DESTINATION lib/${PROJECT_NAME}
)

# 包的描述
ament_package()