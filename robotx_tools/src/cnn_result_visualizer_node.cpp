// headers in this package
#include <cnn_result_visualizer.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cnn_result_visualizer_node");
  cnn_result_visualizer visualizer;
  ros::spin();
  return 0;
}