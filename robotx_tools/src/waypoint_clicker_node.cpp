// headers in this package
#include <waypoint_clicker.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "waypoint_clicker");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  waypoint_clicker clicker(nh,pnh);
  ros::spin();
  return 0;
}