//headers in ros
#include <ros/ros.h>
#include <object_bbox_extractor.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "object_bbox_extractor_node");
  object_bbox_extractor extractor;
  ros::spin();
  return 0;
}