cmake_minimum_required(VERSION 2.8.3)
project(robotx_gazebo)

include(../Doxygen/doxygen.cmake)
include(../Format/ClangFormat.cmake)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  hector_gazebo_plugins
  message_generation
  nav_msgs
  roscpp
  std_msgs
  wamv_description
  robotx_msgs
  tf
)

## Gazebo
find_package(gazebo REQUIRED)

## Eigen
find_package(Eigen3 REQUIRED)

# Visual Studio enables c++11 support by default
if (NOT MSVC)
  set(UNFILTERED_FLAGS "-std=c++11")
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wall -Wextra")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs nav_msgs
  LIBRARIES
)

###########
## Build ##
###########
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

# buoyancy_gazebo_plugin
add_library(buoyancy_gazebo_plugin
  SHARED
  src/buoyancy_gazebo_plugin.cc
 )
target_link_libraries(buoyancy_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
add_document(buoyancy_gazebo_plugin)
clang_format(buoyancy_gazebo_plugin)

# usv_gazebo_dynamics_plugin
add_library(usv_gazebo_dynamics_plugin
  SHARED
  src/usv_gazebo_dynamics_plugin.cc
)
target_link_libraries(usv_gazebo_dynamics_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_document(usv_gazebo_dynamics_plugin)
clang_format(usv_gazebo_dynamics_plugin)

# usv_gazebo_thrust_plugin
add_library(usv_gazebo_thrust_plugin
  SHARED
  src/usv_gazebo_thrust_plugin.cc
)
add_dependencies(usv_gazebo_thrust_plugin robotx_msgs_generate_messages_cpp)
target_link_libraries(usv_gazebo_thrust_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_document(usv_gazebo_thrust_plugin)
clang_format(usv_gazebo_thrust_plugin)

## usv_gazebo_wind_plugin
add_library(usv_gazebo_wind_plugin
  SHARED
  src/usv_gazebo_wind_plugin.cc
)
target_link_libraries(usv_gazebo_wind_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_document(usv_gazebo_wind_plugin)
clang_format(usv_gazebo_wind_plugin)

# usv_twist_sensor_plugin
add_library(usv_twist_sensor_plugin
  SHARED
  src/usv_twist_sensor_plugin.cc
)
target_link_libraries(usv_twist_sensor_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_document(usv_twist_sensor_plugin)
clang_format(usv_twist_sensor_plugin)

# ball_launcher_plugin
add_library(ball_launcher_plugin
  SHARED
  src/ball_launcher_plugin.cc
)
target_link_libraries(ball_launcher_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_document(ball_launcher_plugin)
clang_format(ball_launcher_plugin)

#############
## Install ##
#############
# buoyancy_gazebo_plugin
install(TARGETS buoyancy_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# usv_gazebo_dynamics_plugin
install(TARGETS usv_gazebo_dynamics_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# usv_gazebo_thrust_plugin
install(TARGETS usv_gazebo_thrust_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# usv_gazebo_wind_plugin
install(TARGETS usv_gazebo_wind_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# usv_twist_sensor_plugin
install(TARGETS usv_twist_sensor_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ball_launcher_plugin
install(TARGETS ball_launcher_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# include header files
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Python scripts
set(PYTHON_SCRPTS scripts/twist2drive_diff.py scripts/twist2drive_keyboard.py src/quat2rpy.py)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)

# Install all files
set(MODE_FILES models/ocean models/ocean/materials/programs/GLSL models/ocean/materials/scripts models/ocean/materials/textures models/ocean/meshes models/sandisland models/surmark950400/ models/surmark950400/mesh models/surmark950410/ models/surmark950410/mesh )

foreach(dir config launch ${MODEL_FILES} urdf worlds)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Configure the setup script
#catkin_add_env_hooks(99_robotx_gazebo_setup
#  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
#  SHELLS sh)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
