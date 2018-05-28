cmake_minimum_required(VERSION 2.8.3)
project(robotx_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  robotx_msgs
  tf
  tf2_ros
  pcl_ros
)

catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_from_fix_velocity_node src/twist_from_fix_velocity_node.cpp src/twist_calculator.cpp)
target_link_libraries(twist_from_fix_velocity_node ${catkin_LIBRARIES})

add_executable(robotx_map_server_node src/robotx_map_server_node.cpp src/robotx_map_server.cpp)
target_link_libraries(robotx_map_server_node ${catkin_LIBRARIES})

add_executable(geographic_map_server_node src/geographic_map_server_node.cpp src/geographic_map_server.cpp)
target_link_libraries(geographic_map_server_node ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
