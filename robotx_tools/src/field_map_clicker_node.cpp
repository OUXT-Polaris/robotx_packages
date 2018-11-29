// headers in this package
#include <field_map_clicker.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "field_map_clicker_node");
  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");
  field_map_clicker clicker(nh,pnh);
  ros::spin();
  return 0;
}