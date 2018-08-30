cmake_minimum_required(VERSION 2.8.3)
project(robotx_navigation)

include(../../Doxygen/doxygen.cmake)
include(../../Format/ClangFormat.cmake)

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
  nav_msgs
  jsk_recognition_msgs
  dwa_local_planner
  global_planner
  yaml_cpp_0_3
  pcl_ros
  message_filters
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
add_dependencies(twist_from_fix_velocity_node robotx_msgs_generate_messages_cpp)
add_document(twist_from_fix_velocity_node)
clang_format(twist_from_fix_velocity_node)

add_executable(geographic_map_server_node src/geographic_map_server_node.cpp src/geographic_map_server.cpp)
target_link_libraries(geographic_map_server_node ${catkin_LIBRARIES} -lyaml-cpp)
add_dependencies(geographic_map_server_node robotx_msgs_generate_messages_cpp)
add_document(geographic_map_server_node)
clang_format(geographic_map_server_node)

add_executable(robotx_localization_node src/robotx_localization_node.cpp src/robotx_localization.cpp src/particle_filter.cpp)
target_link_libraries(robotx_localization_node ${catkin_LIBRARIES})
add_dependencies(robotx_localization_node robotx_msgs_generate_messages_cpp)
add_document(robotx_localization_node)
clang_format(robotx_localization_node)

add_executable(obstacle_map_server_node src/obstacle_map_server_node.cpp src/obstacle_map_server.cpp)
target_link_libraries(obstacle_map_server_node ${catkin_LIBRARIES})
add_dependencies(obstacle_map_server_node robotx_msgs_generate_messages_cpp)
add_document(obstacle_map_server_node)
clang_format(obstacle_map_server_node)

add_executable(costmap_clear_request_sender_node src/costmap_clear_request_sender_node.cpp src/costmap_clear_request_sender.cpp)
target_link_libraries(costmap_clear_request_sender_node ${catkin_LIBRARIES})
add_document(costmap_clear_request_sender_node)
clang_format(costmap_clear_request_sender_node)

add_executable(ndt_mapping_node src/ndt_mapping_node.cpp src/ndt_mapping.cpp)
target_link_libraries(ndt_mapping_node ${catkin_LIBRARIES})
add_document(ndt_mapping_node)
clang_format(ndt_mapping_node)

#############
## Install ##
#############
# CPP codes
set(NODE_FILES 
  twist_from_fix_velocity_node 
  geographic_map_server_node 
  obstacle_map_server_node 
  robotx_localization_node 
  costmap_clear_request_sender_node 
  ndt_mapping_node)
foreach(nodes ${NODE_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(nodes)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

# Python scripts
set(PYTHON_SCRPTS 
  scripts/joy_to_cmd.py scripts/motor_command_sender.py scripts/test_cmd_vel_publisher.py scripts/world_frame_publisher.py)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)

# Install all files
foreach(dir launch config data)
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
