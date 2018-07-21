// headers in ros
#include <object_bbox_extractor.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "object_bbox_extractor_node");
  object_bbox_extractor extractor;
  ros::spin();
  return 0;
}