#include <bing_object_detection.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "bing_object_detection");
  bing_object_detection detector;
  ros::spin();
  return 0;
}