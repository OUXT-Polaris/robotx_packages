/**
 * @mainpage メインタイトル
 * いろいろな説明
 */

//headers in this package
#include <heartbeat_publisher.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "heartbeat_publisher_node");
  heartbeat_publisher publisher;
  ros::spin();
  return 0;
}
