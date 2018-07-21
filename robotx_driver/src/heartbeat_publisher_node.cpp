/**
 * @mainpage heartbeat_publisher_node
 * ROS node for publish robotx heartbeat protocol in TCP/IP
 * @author Masaya Kataoka
 * @date 2018.06.09
 * @image html images/logo.jpg
 */

// headers in this package
#include <heartbeat_publisher.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "heartbeat_publisher_node");
  heartbeat_publisher publisher;
  ros::spin();
  return 0;
}
