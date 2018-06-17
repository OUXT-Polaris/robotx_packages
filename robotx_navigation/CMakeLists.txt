cmake_minimum_required(VERSION 2.8.3)
project(robotx_navigation)

include(../Doxygen/doxygen.cmake)

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
  tf2_geometry_msgs
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
add_document(twist_from_fix_velocity_node)

add_executable(geographic_map_server_node src/geographic_map_server_node.cpp src/geographic_map_server.cpp)
target_link_libraries(geographic_map_server_node ${catkin_LIBRARIES})
add_document(geographic_map_server_node)

#############
## Install ##
#############
# CPP codes
set(NODE_FILES twist_from_fix_velocity_node geographic_map_server_node)
foreach(nodes ${NODE_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(nodes)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

# Python scripts
set(PYTHON_SCRPTS scripts/joy_to_cmd.py scripts/motor_command_sender.py scripts/test_cmd_vel_publisher.py scripts/world_frame_publisher.py)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)

# Install all files
foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
