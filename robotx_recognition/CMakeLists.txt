cmake_minimum_required(VERSION 2.8.3)
project(robotx_recognition)

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
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  DEPENDS Eigen
)

###########
## Build ##
###########
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
set(object_recognition_src src/object_recognition_node.cpp src/pcl_object_recognition.cpp src/object_model.cpp)
add_executable(object_recognition_node ${object_recognition_src})
add_dependencies(object_recognition_node robotx_msgs_generate_messages_cpp)

set(euclidean_clustering_src src/euclidean_clustering_node.cpp src/euclidean_clustering.cpp)
add_executable(euclidean_clustering_node ${euclidean_clustering_src})
add_dependencies(euclidean_clustering_node robotx_msgs_generate_messages_cpp)

set(hsv_buoy_detector_src src/hsv_buoy_detector_node.cpp src/hsv_buoy_detector.cpp)
add_executable(hsv_buoy_detector_node ${hsv_buoy_detector_src})
add_dependencies(hsv_buoy_detector_node robotx_msgs_generate_messages_cpp)

set(bing_object_detection_node_src src/bing_object_detection_node.cpp src/bing_object_detection.cpp)
add_executable(bing_object_detection_node ${bing_object_detection_node_src})
add_dependencies(bing_object_detection_node robotx_msgs_generate_messages_cpp)

set(stereo_image_object_bbox_extractor_src src/disparity_image.cpp src/stereo_image_object_bbox_extractor.cpp src/random_sampling.cpp)
add_executable(stereo_image_object_bbox_extractor_node src/stereo_image_object_bbox_extractor_node.cpp ${stereo_image_object_bbox_extractor_src})
add_dependencies(stereo_image_object_bbox_extractor_node robotx_msgs_generate_messages_cpp)

set(passthrough_filter_src src/passthrough_filter.cpp)
add_executable(passthrough_filter_node src/passthrough_filter_node.cpp ${passthrough_filter_src})
add_dependencies(passthrough_filter_node robotx_msgs_generate_messages_cpp)

set(pointcloud_merger_src src/pointcloud_merger.cpp)
add_executable(pointcloud_merger_node src/pointcloud_merger_node.cpp ${pointcloud_merger_src})
add_dependencies(pointcloud_merger_node robotx_msgs_generate_messages_cpp)

set(euclidean_cluster_tracker_node_src src/euclidean_cluster_tracker_node.cpp src/particle_filter.cpp src/euclidean_cluster_tracker.cpp)
add_executable(euclidean_cluster_tracker_node ${euclidean_cluster_tracker_node_src})
add_dependencies(euclidean_cluster_tracker_node robotx_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(object_recognition_node ${catkin_LIBRARIES})
target_link_libraries(euclidean_clustering_node ${catkin_LIBRARIES})
target_link_libraries(hsv_buoy_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(bing_object_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(stereo_image_object_bbox_extractor_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(passthrough_filter_node ${catkin_LIBRARIES})
target_link_libraries(pointcloud_merger_node ${catkin_LIBRARIES})
target_link_libraries(euclidean_cluster_tracker_node ${catkin_LIBRARIES})

#############
## Install ##
#############
# CPP Execution programs
set(CPP_EXEC_NAMES 
  object_recognition_node euclidean_clustering_node passthrough_filter_node pointcloud_merger_node
  hsv_buoy_detector_node bing_object_detection_node stereo_image_object_bbox_extractor_node)
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
