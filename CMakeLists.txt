cmake_minimum_required(VERSION 3.0.2)
project(line_rot_slam)

set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp std_msgs geometry_msgs)

# Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# Create Cpp executable
add_executable(twist_keyboard_control_node src/twist_keyboard_control.cpp)
target_link_libraries(twist_keyboard_control_node ${catkin_LIBRARIES})

# Install Cpp executable
install(TARGETS twist_keyboard_control_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
