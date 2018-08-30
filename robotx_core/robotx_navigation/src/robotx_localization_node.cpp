// headers in this package
#include <robotx_localization.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robotx_localization_node");
  robotx_localization localization;
  ros::spin();
  return 0;
}