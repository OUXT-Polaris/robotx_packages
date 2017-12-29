//headers in this package
#include <robotx_map_server.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robotx_map_server_node");
  robotx_map_server map_server;
  ros::spin();
  return 0;
}
