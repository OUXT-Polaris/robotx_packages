// headers in this package
#include <navi_sim.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "navi_sim");
  navi_sim sim;
  sim.run();
  ros::spin();
  return 0;
}