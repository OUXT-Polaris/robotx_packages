// headers in this package
#include <cmd_vel_visualizer.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cmd_vel_visualizer_node");
  cmd_vel_visualizer visualizer;
  ros::spin();
  return 0;
}