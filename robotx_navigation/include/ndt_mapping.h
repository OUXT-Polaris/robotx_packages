#ifndef NDT_MAPPING_H_INCLUDED
#define NDT_MAPPING_H_INCLUDED

// headers in ROS
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class ndt_mapping {
 public:
  struct parameters {
    std::string pointcloud_topic;
    std::string odom_topic;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/pointcloud_topic", pointcloud_topic,
                                     ros::this_node::getName() + "/input_pointcloud");
      ros::param::param<std::string>(ros::this_node::getName() + "/odom_topic", odom_topic, "/odom");
    }
  };
  ndt_mapping();
  ~ndt_mapping();

 private:
  const parameters params_;
  ros::NodeHandle nh_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
  message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync_;
  void callback_(const nav_msgs::OdometryConstPtr& odom_msg,
                 const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg);
};
#endif  // NDT_MAPPING_H_INCLUDED