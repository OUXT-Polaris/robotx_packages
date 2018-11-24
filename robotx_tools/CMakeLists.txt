cmake_minimum_required(VERSION 2.8.3)
project(robotx_tools)

include(../Format/ClangFormat.cmake)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  rviz
  robotx_msgs
  tf
)

find_package(PythonLibs)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread system chrono)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate services in the 'srv' folder
add_service_files(
  FILES
  PlotCharacteristicCurve.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
#   geometry_msgs#   sensor_msgs#   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  DEPENDS rviz
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS -g)
set(CMAKE_AUTOMOC ON)

set(cnn_result_visualizer_src src/cnn_result_visualizer.cpp)
add_executable(cnn_result_visualizer_node src/cnn_result_visualizer_node.cpp ${cnn_result_visualizer_src})
add_dependencies(cnn_result_visualizer_node robotx_msgs_generate_messages_cpp ${catkin_EXPORTED_LIBRARIES})
target_link_libraries(cnn_result_visualizer_node ${catkin_LIBRARIES})
clang_format(cnn_result_visualizer_node)

set(fake_technical_director_network_src src/fake_technical_director_network_main.cpp src/tcp_server.cpp)
add_executable(fake_technical_director_network ${fake_technical_director_network_src})
target_link_libraries(fake_technical_director_network ${Boost_LIBRARIES})
clang_format(fake_technical_director_network)

add_library(map_editor_plugin
  src/plant_green_buoy.cpp include/plant_green_buoy.h
  src/plant_red_buoy.cpp include/plant_red_buoy.h
  src/plant_white_buoy.cpp include/plant_white_buoy.h
  src/lat_lon_ruler.cpp include/lat_lon_ruler.h include/UTM.h src/UTM.cpp)
target_link_libraries(map_editor_plugin ${catkin_LIBRARIES} ${QT_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})

#############
## Install ##
#############
# CPP plugin
install(TARGETS cnn_result_visualizer_node map_editor_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# Include files
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Python scripts
set(PYTHON_SCRIPTS scripts/gps_plotter.py scripts/kml_plotter.py scripts/map_downloader.py scripts/objects_2d_viewer.py scripts/plot_characteristic_curve.py)
foreach(scripts ${PYTHON_SCRIPTS})
  catkin_install_python(PROGRAMS ${scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(scripts)

# All files
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

foreach(dir config icons/classes launch resource srv)
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
