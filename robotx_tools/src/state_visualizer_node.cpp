// headers in this package
#include <state_visualizer.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "state_visualizer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  state_visualizer visualizer(nh,pnh);
  ros::spin();
  return 0;
}