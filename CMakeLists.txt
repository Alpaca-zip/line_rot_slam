cmake_minimum_required(VERSION 3.0.2)
project(line_rot_slam)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  message_runtime
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
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_generation
    message_runtime
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
  src/map_optimization_nodelet.cpp
  src/pcd_process_nodelet.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(twist_keyboard_control_node src/twist_keyboard_control.cpp)
target_link_libraries(twist_keyboard_control_node ${catkin_LIBRARIES})

add_executable(noisy_odom_publisher_node src/noisy_odom_publisher.cpp)
target_link_libraries(noisy_odom_publisher_node ${catkin_LIBRARIES})

install(TARGETS twist_keyboard_control_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS noisy_odom_publisher_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
