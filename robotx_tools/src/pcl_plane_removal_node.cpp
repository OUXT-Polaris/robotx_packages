// headers in this package
#include <pcl_plane_removal.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pcl_plane_removal_node");
  pcl_plane_removal plane_removal;
  ros::spin();
  return 0;
}