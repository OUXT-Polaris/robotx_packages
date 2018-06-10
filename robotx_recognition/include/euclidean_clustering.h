#ifndef EUCLIDEAN_CLUSTERING_H_INCLUDED
#define EUCLIDEAN_CLUSTERING_H_INCLUDED

//headers in ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

//headers in pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

class euclidean_clustering
{
public:
  euclidean_clustering();
  ~euclidean_clustering();
private:
  bool check_bbox_size(geometry_msgs::Vector3 bbox_scale);
  void poincloud_callback(sensor_msgs::PointCloud2 msg);
  void make_cluster(sensor_msgs::PointCloud2 msg);
  static bool custom_region_growing_function(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance);
  static bool enforce_intensity_similarity(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance);
  ros::Publisher marker_pub_;
  ros::Subscriber pointcloud_sub_;
  ros::NodeHandle nh_;
  //parameters
  int min_cluster_size_,max_cluster_size_;
  double cluster_tolerance_,leaf_size_x,leaf_size_y,leaf_size_z,radius_search_;
  double min_bbox_size_;
  double max_bbox_size_;
  std::string input_cloud_;
};
#endif  //EUCLIDEAN_CLUSTERING_H_INCLUDED
