// headers in this package
#include <hsv_buoy_detector.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hsv_buoy_detector_node");
  hsv_buoy_detector detector();
  ros::spin();
  return 0;
}
