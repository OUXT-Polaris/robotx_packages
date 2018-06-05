//headers in this package
//#include <euclidean_clustering.h>

//headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robotx_euclidean_cluster_tracker_node");
  ros::spin();
  return 0;
}