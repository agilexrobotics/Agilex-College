cmake_minimum_required(VERSION 3.10)
project(cubeAndLineDet)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_BUILD_TYPE Debug)

set(PCL_DIR "/usr/local/share/pcl-1.10")

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

find_package(PCL 1.10 REQUIRED COMPONENTS common io surface visualization)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include/pcl-1.10  
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
  DEPENDS 
  PCL
  OpenCV
)

add_executable(cube_det src/cube_det.cpp)
target_link_libraries(cube_det
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  pcl_surface 
)

add_executable(line_det src/line_det.cpp)
target_link_libraries(line_det
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  pcl_surface 
)
