#ifndef VOXELCHAIN_INPUT_GENERATOR_H_INCLUDED
#define VOXELCHAIN_INPUT_GENERATOR_H_INCLUDED

//headers in this package
#include <robotx_msgs/EuclideanClusters.h>

//headers in ROS
#include <ros/ros.h>

class voxelchain_input_generator
{
public:
  voxelchain_input_generator();
  ~voxelchain_input_generator();
  void euclidean_clusters_callback(robotx_msgs::EuclideanClusters msg);
private:
  ros::Subscriber euclidean_clusters_sub_;
  ros::NodeHandle nh_;
};
#endif  //VOXELCHAIN_INPUT_GENERATOR_H_INCLUDED
