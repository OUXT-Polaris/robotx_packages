cmake_minimum_required(VERSION 2.8.3)
project(robotx_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Object2D.msg
  Objects2D.msg
  Object.msg
  Objects.msg
  Heartbeat.msg
  TechnicalDirectorNetworkStatus.msg
  BoundingBox2D.msg
  BoundingBox3D.msg
  EuclideanClusters.msg
  VoxelChainInput.msg
  VoxelChainInputs.msg
  UsvDrive.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#
  sensor_msgs#
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
)

###########
## Build ##
###########

# Nothing to build

#############
## Install ##
#############
foreach(dir msg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############

# Nothing to test
