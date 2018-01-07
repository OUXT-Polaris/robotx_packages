#ifndef EUCLIDEAN_CLUSTERING_H_INCLUDED
#define EUCLIDEAN_CLUSTERING_H_INCLUDED

//headers in ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//headers in pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

class euclidean_clustering
{
public:
  euclidean_clustering();
  ~euclidean_clustering();
private:
  void poincloud_callback(sensor_msgs::PointCloud2 msg);
  void make_cluster();
  ros::Publisher pointcloud_pub_,clusters_pub_;
  ros::Subscriber pointcloud_sub_;
  ros::NodeHandle nh_;
  sensor_msgs::PointCloud2 pointcloud_;
  //parameters
  int min_cluster_size_,max_cluster_size_;
  double cluster_tolerance_,leaf_size_x,leaf_size_y,leaf_size_z,radius_search_;
  std::string input_cloud_;
};
#endif  //EUCLIDEAN_CLUSTERING_H_INCLUDED
