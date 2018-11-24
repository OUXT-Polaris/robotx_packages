// headers in this package
#include <map_saver.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "map_saver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  map_saver saver(nh,pnh);
  ros::spin();
  return 0;
}