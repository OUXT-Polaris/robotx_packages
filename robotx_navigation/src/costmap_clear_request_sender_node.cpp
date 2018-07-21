// headers in this package
#include <costmap_clear_request_sender.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "costmap_clear_request_sender_node");
  costmap_clear_request_sender sender;
  sender.run();
  ros::spin();
  return 0;
}
