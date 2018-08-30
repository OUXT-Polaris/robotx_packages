cmake_minimum_required(VERSION 2.8.3)
project(robotx_driver)
include(../../Doxygen/doxygen.cmake)
include(../../Format/ClangFormat.cmake)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  robotx_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  diagnostic_msgs
  diagnostic_updater
)

find_package(Boost REQUIRED COMPONENTS thread system chrono)

find_package(CUDA)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotx_driver
  CATKIN_DEPENDS joy robotx_msgs roscpp rospy sensor_msgs std_msgs diagnostic_msgs diagnostic_updater
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if(CUDA_FOUND)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )
else()
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
endif()

set(heartbeat_publisher_node_src src/heartbeat_publisher_node.cpp src/heartbeat_publisher.cpp src/tcp_client.cpp)
add_executable(heartbeat_publisher_node ${heartbeat_publisher_node_src})
add_dependencies(heartbeat_publisher_node robotx_msgs_generate_messages_cpp)
target_link_libraries(heartbeat_publisher_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_document(heartbeat_publisher_node)
clang_format(heartbeat_publisher_node)

set(robotx_hardware_interface_src 
  src/robotx_hardware_interface_node.cpp src/robotx_hardware_interface.cpp src/tcp_client.cpp src/remote_operated_interface.cpp)
add_executable(robotx_hardware_interface_node ${robotx_hardware_interface_src} src/robotx_hardware_interface_node.cpp)
add_dependencies(robotx_hardware_interface_node robotx_msgs_generate_messages_cpp)
target_link_libraries(robotx_hardware_interface_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_document(robotx_hardware_interface_node)
clang_format(robotx_hardware_interface_node)

message(STATUS "")
message(STATUS "FoundCUDA              : ${CUDA_FOUND}")

if(CUDA_FOUND)
  #set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -std=c++11 -gencode arch=compute_30,code=sm_30)
  #set(CUDA_SEPARABLE_COMPILATION ON)
  #set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  #set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}; "-std=c++11 -O3")
  set(CUDA_HOST_COMPILER g++)

  set(cuda_diagnostic_src src/cuda_diagnostic.cpp)
  cuda_add_executable(cuda_diagnostic_node ${cuda_diagnostic_src} src/cuda_diagnostic_node.cpp)
  add_dependencies(cuda_diagnostic_node robotx_msgs_generate_messages_cpp)
  target_link_libraries(cuda_diagnostic_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIBRARIES})
  add_document(cuda_diagnostic_node)
  clang_format(cuda_diagnostic_node)
endif()


#############
## Install ##
#############
if(CUDA_FOUND)
  install(TARGETS cuda_diagnostic_node robotx_hardware_interface_node heartbeat_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  install(TARGETS robotx_hardware_interface_node heartbeat_publisher_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

foreach(dir config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if(CUDA_FOUND)
  foreach(dir launch launch_cuda)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endforeach(dir)
else()
  foreach(dir launch)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endforeach(dir)
endif()

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()