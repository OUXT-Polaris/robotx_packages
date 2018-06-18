cmake_minimum_required(VERSION 2.8.3)
project(wamv_description)

find_package(catkin REQUIRED COMPONENTS
  hector_gazebo_plugins
)

catkin_package()

###########
## Build ##
###########

# Nothing to build

#############
## Install ##
#############
set(URDF_DIRS urdf urdf/base urdf/data urdf/sensors urdf/sensors/camera urdf/sensors/gps urdf/sensors/imu 
  urdf/sensors/twist urdf/sensors/velodyne urdf/thruster urdf/sensors/stereo_camera)
foreach(dir launch mesh/base ${URDF_DIRS})
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
