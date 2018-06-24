/**
 * @mainpage ping_sender_node
 * ROS node for sending ping
 * @author Masaya Kataoka
 * @date 2018.06.24
 * @image html images/logo.jpg
 */

//headers in this package
#include <ping_sender.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ping_sender_node");
  ping_sender sender;
  ros::spin();
  return 0;
}
