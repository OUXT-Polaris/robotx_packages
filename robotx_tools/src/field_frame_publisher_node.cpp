#include <field_frame_publisher.h>
// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "field_frame_publisher");
  field_frame_publisher publisher;
  ros::spin();
  return 0;
}