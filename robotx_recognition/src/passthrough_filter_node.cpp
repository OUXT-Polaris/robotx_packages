//headers in this package
#include <passthrough_filter.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "passthrough_filter_node");
  passthough_filter filter();
  ros::spin();
  return 0;
}