#ifndef EUCLIDEAN_CLUSTERING_H_INCLUDED
#define EUCLIDEAN_CLUSTERING_H_INCLUDED

// headers in ros
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// headers in pcl
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

class euclidean_clustering {
 public:
  struct euclidean_clustering_parameters {
    double cluster_tolerance;
    euclidean_clustering_parameters() {
      ros::param::param<double>(ros::this_node::getName() + "/euclidean_clustering/cluster_tolerance",
                                cluster_tolerance, 0.1);
    }
  };
  struct conditional_euclidian_clustering_parameters {
    double radius_search;
    double cluster_tolerance;
    conditional_euclidian_clustering_parameters() {
      ros::param::param<double>(ros::this_node::getName() + "/conditional_euclidean_clustering/radius_search",
                                radius_search, 50);
      ros::param::param<double>(
          ros::this_node::getName() + "/conditional_euclidean_clustering/cluster_tolerance",
          cluster_tolerance, 1);
    }
  };
  struct bbox_parameters {
    double inflation_size;
    bbox_parameters() {
      ros::param::param<double>(ros::this_node::getName() + "/inflation_size", inflation_size, 1);
    }
  };
  euclidean_clustering();
  ~euclidean_clustering();
  enum clustering_methods { CONDITIONAL_EUCLIDIAN_CLUSTERING = 0, EUCLIDIAN_CLUSTER_EXTRACTION = 1 };

 private:
  const conditional_euclidian_clustering_parameters conditional_euclidian_clustering_params_;
  const euclidean_clustering_parameters euclidian_clustering_params_;
  const bbox_parameters bbox_params_;
  void poincloud_callback(sensor_msgs::PointCloud2 msg);
  void make_cluster(sensor_msgs::PointCloud2 msg);
  static bool custom_region_growing_function(const pcl::PointXYZINormal& point_a,
                                             const pcl::PointXYZINormal& point_b,
                                             float squared_distance);
  static bool enforce_intensity_similarity(const pcl::PointXYZINormal& point_a,
                                           const pcl::PointXYZINormal& point_b,
                                           float squared_distance);
  ros::Publisher marker_pub_;
  ros::Subscriber pointcloud_sub_;
  ros::NodeHandle nh_;
  // parameters
  int min_cluster_size_, max_cluster_size_;
  int clustering_method_;
  double leaf_size_x, leaf_size_y, leaf_size_z;
  bool donwsample_;
  std::string input_cloud_;
};
#endif  // EUCLIDEAN_CLUSTERING_H_INCLUDED
