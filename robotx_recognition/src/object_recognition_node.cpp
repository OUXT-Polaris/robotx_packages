// headers in this package
#include <pcl_object_recognition.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robotx_object_recognition_node");
  pcl_object_recognition object_recognition;
  ros::spin();
  return 0;
}
