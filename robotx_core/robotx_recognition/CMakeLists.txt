cmake_minimum_required(VERSION 2.8.3)
project(robotx_recognition)

include(../../Doxygen/doxygen.cmake)
include(../../Format/ClangFormat.cmake)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  robotx_msgs
  image_transport
  cv_bridge
  visualization_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  jsk_recognition_msgs
  nodelet
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# object_recognition
set(object_recognition_src src/pcl_object_recognition.cpp src/object_model.cpp)
add_executable(object_recognition_node src/object_recognition_node.cpp ${object_recognition_src})
add_dependencies(object_recognition_node robotx_msgs_generate_messages_cpp)
target_link_libraries(object_recognition_node ${catkin_LIBRARIES})
clang_format(object_recognition_node)
add_library(object_recognition_nodelet src/object_recognition_nodelet.cpp ${object_recognition_src})
add_dependencies(object_recognition_nodelet robotx_msgs_generate_messages_cpp ${catkin_EXPORTED_LIBRARIES})
target_link_libraries(object_recognition_nodelet ${catkin_LIBRARIES})
clang_format(object_recognition_nodelet)

# euclidean_clustering
set(euclidean_clustering_src src/euclidean_clustering.cpp)
add_executable(euclidean_clustering_node src/euclidean_clustering_node.cpp ${euclidean_clustering_src})
add_dependencies(euclidean_clustering_node robotx_msgs_generate_messages_cpp)
target_link_libraries(euclidean_clustering_node ${catkin_LIBRARIES})
clang_format(euclidean_clustering_node)
add_library(euclidean_clustering_nodelet src/euclidean_clustering_nodelet.cpp ${euclidean_clustering_src})
add_dependencies(euclidean_clustering_nodelet robotx_msgs_generate_messages_cpp ${catkin_EXPORTED_LIBRARIES})
target_link_libraries(euclidean_clustering_nodelet ${catkin_LIBRARIES})
clang_format(euclidean_clustering_nodelet)

# hsv buoy detector
set(hsv_buoy_detector_src src/hsv_buoy_detector.cpp)
add_executable(hsv_buoy_detector_node src/hsv_buoy_detector_node.cpp ${hsv_buoy_detector_src})
add_dependencies(hsv_buoy_detector_node robotx_msgs_generate_messages_cpp)
target_link_libraries(hsv_buoy_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
clang_format(hsv_buoy_detector_node)

# bing object detection
set(bing_object_detection_node_src src/bing_object_detection.cpp)
add_executable(bing_object_detection_node src/bing_object_detection_node.cpp ${bing_object_detection_node_src})
add_dependencies(bing_object_detection_node robotx_msgs_generate_messages_cpp)
target_link_libraries(bing_object_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
clang_format(bing_object_detection_node)

# object bbox extractor
set(object_bbox_extractor_src src/object_bbox_extractor.cpp)
add_executable(object_bbox_extractor_node src/object_bbox_extractor_node.cpp ${object_bbox_extractor_src})
add_dependencies(object_bbox_extractor_node robotx_msgs_generate_messages_cpp)
target_link_libraries(object_bbox_extractor_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
clang_format(object_bbox_extractor_node)
add_library(object_bbox_extractor_nodelet src/object_bbox_extractor_nodelet.cpp ${object_bbox_extractor_src})
add_dependencies(object_bbox_extractor_nodelet robotx_msgs_generate_messages_cpp)
target_link_libraries(object_bbox_extractor_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBS})
clang_format(object_bbox_extractor_nodelet)

# passthrough_filter
set(passthrough_filter_src src/passthrough_filter.cpp)
add_executable(passthrough_filter_node src/passthrough_filter_node.cpp ${passthrough_filter_src})
add_dependencies(passthrough_filter_node robotx_msgs_generate_messages_cpp)
target_link_libraries(passthrough_filter_node ${catkin_LIBRARIES})
clang_format(passthrough_filter_node)
add_library(passthrough_filter_nodelet src/passthrough_filter_nodelet.cpp ${passthrough_filter_src})
add_dependencies(passthrough_filter_nodelet robotx_msgs_generate_messages_cpp)
target_link_libraries(passthrough_filter_nodelet ${catkin_LIBRARIES})
clang_format(passthrough_filter_nodelet)

# pointcloud_merger
set(pointcloud_merger_src src/pointcloud_merger.cpp)
add_executable(pointcloud_merger_node src/pointcloud_merger_node.cpp ${pointcloud_merger_src})
add_dependencies(pointcloud_merger_node robotx_msgs_generate_messages_cpp)
target_link_libraries(pointcloud_merger_node ${catkin_LIBRARIES})
clang_format(pointcloud_merger_node)
add_library(pointcloud_merger_nodelet src/pointcloud_merger_nodelet.cpp ${pointcloud_merger_src})
add_dependencies(pointcloud_merger_nodelet robotx_msgs_generate_messages_cpp)
target_link_libraries(pointcloud_merger_nodelet ${catkin_LIBRARIES})
clang_format(pointcloud_merger_nodelet)

#############
## Install ##
#############
# CPP Execution programs
set(CPP_EXEC_NAMES 
  object_recognition_node euclidean_clustering_node passthrough_filter_node pointcloud_merger_node
  hsv_buoy_detector_node bing_object_detection_node object_bbox_extractor_node)
foreach(cpp_exec_names ${CPP_EXEC_NAMES})
  install(TARGETS ${cpp_exec_names}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(cpp_exec_names)
# include header files
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install all files
foreach(dir config data launch)
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
