cmake_minimum_required(VERSION 3.0.2)
project(line_rot_slam)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  cv_bridge
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_action_files(
  FILES
  Optimization.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    cv_bridge
    pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pcd_process_nodelet.cpp
  src/map_optimization_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(twist_keyboard_control_node src/twist_keyboard_control.cpp)
target_link_libraries(twist_keyboard_control_node ${catkin_LIBRARIES})

install(TARGETS twist_keyboard_control_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
