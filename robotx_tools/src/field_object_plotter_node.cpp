#include <field_object_plotter.h>
// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "field_object_plotter_node");
  field_object_plotter plotter;
  ros::spin();
  return 0;
}