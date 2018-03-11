//headers in this package
#include <hsv_buoy_detector.h>

//headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "hsv_buoy_detector_node");
  ros::spin();
  return 0;
}
