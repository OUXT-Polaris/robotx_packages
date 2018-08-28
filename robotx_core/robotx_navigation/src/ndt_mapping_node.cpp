// headers in this package
#include <ndt_mapping.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ndt_mapping_node");
  ndt_mapping mapping;
  ros::spin();
  return 0;
}