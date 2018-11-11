#ifndef PCL_PLANE_REMOVAL_H_INCLUDED
#define PCL_PLANE_REMOVAL_H_INCLUDED

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

// headers in ROS
#include <ros/ros.h>

// headers in STL
#define _USE_MATH_DEFINES
#include <cmath>

class pcl_plane_removal {
 public:
  struct parameters {
    std::string input_pcl_topic,output_pcl_topic;
    double DistanceThreshold;//RANSACの最小二乗法の許容誤差範囲
    int MaxIterations;
    double Probability;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/input_pcl_topic", input_pcl_topic,
                                     ros::this_node::getName() + "/input_pcl");
      ros::param::param<std::string>(ros::this_node::getName() + "/output_pcl_topic", output_pcl_topic,
                                     ros::this_node::getName() + "/output_pcl");
      ros::param::param<double>(ros::this_node::getName() + "/DistanceThreshold", DistanceThreshold,0.1);
      ros::param::param<int>(ros::this_node::getName() + "/MaxIterations", MaxIterations,100);
      ros::param::param<double>(ros::this_node::getName() + "/Probability", Probability ,0.95);
    }
  };
  pcl_plane_removal();
  ~pcl_plane_removal();

 private:
  const parameters params_;
  ros::NodeHandle nh_;
  ros::Publisher modified_pcl_pub_;
  ros::Subscriber pcl_input_sub_;

  void cloud_CB(const sensor_msgs::PointCloud2ConstPtr& msg);
};
#endif  // CNN_RESULT_VISUALIZER_H_INCLUDED