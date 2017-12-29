#ifndef ROBOTX_MAP_SERVER_H_INCLUDED
#define ROBOTX_MAP_SERVER_H_INCLUDED

//headers in stl
#include <string>
#include <vector>

//headers in ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

//headers in boost
//#include <boost/circular_buffer.hpp>

//headers in tf2
#include <tf2_ros/transform_listener.h>

//headers in pcl
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/projection_matrix.h>

class robotx_map_server
{
public:
  robotx_map_server();
  ~robotx_map_server();
private:
  ros::NodeHandle nh_;
  std::string poincloud_topic_;
  ros::Subscriber pointcloud_sub_;
  //int history_length_;
  //boost::circular_buffer
  void pointcloud_callback(sensor_msgs::PointCloud2 pcl_input_cloud);
  void create_map_meta_data();
  void input_map_data(std::vector<pcl::ModelCoefficients::Ptr> coefficients_buoy);
  //transform related members
  tf2_ros::Buffer* tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;
  //map related members
  nav_msgs::OccupancyGrid map_data_;
  ros::Publisher map_pub_;
  int map_width_,map_height_;
  double map_resolution_;
  //buoy detection related members
  std::vector<pcl::ModelCoefficients::Ptr> coefficients_buoy_;
  pcl::ModelCoefficients::Ptr segment_buoy(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
};
#endif //ROBOTX_MAP_SERVER_H_INCLUDED
