cmake_minimum_required(VERSION 3.10)
project(piper_kinematics)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  visualization_msgs
  Eigen3 REQUIRED
  piper_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  visualization_msgs
  piper_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(piper_ik_node src/piper_ik_node.cpp)
target_link_libraries(piper_ik_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(piper_ik_node_use_yaik src/piper_ik_node_use_yaik.cpp)
target_link_libraries(piper_ik_node_use_yaik
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
