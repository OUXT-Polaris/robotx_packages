// headers in this package
#include <obstacle_map_server.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "obstacle_map_server_node");
  obstacle_map_server server;
  ros::spin();
  return 0;
}