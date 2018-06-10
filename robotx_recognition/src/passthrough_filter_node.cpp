//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "passthrough_filter_node");
  ros::spin();
  return 0;
}