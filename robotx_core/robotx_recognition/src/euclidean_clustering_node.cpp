// headers in this package
#include <euclidean_clustering.h>

// headers in ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robotx_euclidean_clustering_node");
  euclidean_clustering clustering;
  ros::spin();
  return 0;
}
