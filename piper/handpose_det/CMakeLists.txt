cmake_minimum_required(VERSION 3.10)
project(handpose_det)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  vision_msgs
  image_geometry
  pcl_conversions
  pcl_ros
  message_filters
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  sensor_msgs 
  image_transport 
  cv_bridge
  vision_msgs
  image_geometry
  pcl_conversions
  pcl_ros
  message_filters
)

find_package(OpenCV REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(handpose_det src/handpose_det.cpp)
target_link_libraries(handpose_det
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
    ${OpenCV_LIBS}
)
