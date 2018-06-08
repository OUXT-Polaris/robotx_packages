cmake_minimum_required(VERSION 2.8.3)
project(robotx_communication)
add_compile_options(-std=c++11)

include(../Doxygen/doxygen.cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  robotx_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(heartbeat_publisher_node_src src/heartbeat_publisher_node.cpp src/heartbeat_publisher.cpp src/tcp_client.cpp)

add_executable(heartbeat_publisher_node ${heartbeat_publisher_node_src})
add_dependencies(heartbeat_publisher_node robotx_msgs_generate_messages_cpp)
add_document(heartbeat_publisher_node)

target_link_libraries(heartbeat_publisher_node ${catkin_LIBRARIES} ${boost_LIBRARIES})

#############
## Install ##
#############
install(TARGETS heartbeat_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
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
