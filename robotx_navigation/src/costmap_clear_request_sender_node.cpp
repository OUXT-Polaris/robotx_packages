//headers in this package
#include <costmap_clear_request_sender.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "costmap_clear_request_sender_node");
  ros::spin();
  return 0;
}
