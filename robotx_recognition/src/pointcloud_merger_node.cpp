//headers in this package
#include <pointcloud_merger.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pointcloud_merger_node");
  ros::spin();
  return 0;
}