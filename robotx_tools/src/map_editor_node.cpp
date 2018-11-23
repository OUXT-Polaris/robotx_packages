// headers in this package
#include <map_editor.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cnn_result_visualizer_node");
  ros::spin();
  return 0;
}