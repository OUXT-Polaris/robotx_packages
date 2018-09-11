cmake_minimum_required(VERSION 2.8.3)
project(wamv_gazebo)
find_package(catkin REQUIRED COMPONENTS
  wamv_description
  usv_gazebo_plugins
  hector_gazebo_plugins
  xacro
)
catkin_package(
  CATKIN_DEPENDS
  wamv_description
  usv_gazebo_plugins
  xacro
  hector_gazebo_plugins
)

# Generate urdf files from xacro and install
xacro_add_files(
  urdf/wamv_basic.gazebo.urdf.xacro
  urdf/wamv_robotx.gazebo.urdf.xacro
  INORDER INSTALL DESTINATION urdf
)

#############
## Install ##
#############
foreach(dir launch urdf)
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
