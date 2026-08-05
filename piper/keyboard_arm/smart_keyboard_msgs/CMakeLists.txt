cmake_minimum_required(VERSION 3.8)
project(smart_keyboard_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UserState.msg"
  "msg/KeyboardTarget.msg"
  "msg/KeyboardStatus.msg"
  "srv/SetKeyboardMode.srv"
  "srv/EmergencyStop.srv"
  "action/MoveKeyboard.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
