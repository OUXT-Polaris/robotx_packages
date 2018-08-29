#ifndef POINTCLOUD_MERGER_H_INCLUDED
#define POINTCLOUD_MERGER_H_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

/**
 * @brief pointcloud_merger class
 *
 * merge 2 pointclouds and output as one pointcloud;
 *
 */
class pointcloud_merger {
 public:
  /**
   * @brief parameters for pointcloud_merger class
   *
   */
  struct parameters {
    std::string pointcloud1_topic;
    std::string pointcloud2_topic;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/pointcloud1_topic", pointcloud1_topic,
                                     ros::this_node::getName() + "/pointcloud1");
      ros::param::param<std::string>(ros::this_node::getName() + "/pointcloud2_topic", pointcloud2_topic,
                                     ros::this_node::getName() + "/pointcloud2");
    }
  };
  pointcloud_merger();
  ~pointcloud_merger();

 private:
  const parameters params_;
  /**
   * @brief tf buffer fir tf_listener_
   * @sa pointcloud_merger::tf_listener_
   */
  tf2_ros::Buffer tf_buffer_;
  /**
   * @brief tf listener for
   * output_frame -> pointcloud1_topic frame
   * and
   * output_frame -> pointcloud2_topic frame
   */
  tf2_ros::TransformListener tf_listener_;
  /**
   * @brief ROS publisher for ~/merged_points (message type : sensor_msgs/PointCloud2)
   *
   */
  ros::Publisher output_pointcloud_pub_;
  /**
   * @brief ROS subscriber for (pointcloud1_topic) topic
   * @sa pointcloud_merger::params_
   */
  ros::Subscriber pointcloud1_sub_;
  /**
   * @brief parameter for pointcloud1
   *
   */
  sensor_msgs::PointCloud2 pointcloud1_msg_;
  /**
   * @brief ROS subscriber for (pointcloud2_topic) topic
   * @sa pointcloud_merger::params_
   */
  ros::Subscriber pointcloud2_sub_;
  /**
   * @brief parameter for pointcloud2
   *
   */
  sensor_msgs::PointCloud2 pointcloud2_msg_;
  /**
   * @brief ROS Nodehandle
   *
   */
  ros::NodeHandle nh_;
  /**
   * @brief ROS callback function for pointcloud1_topic (message type : sensor_msgs/PointCloud2)
   *
   * @param msg ROS message
   * @sa pointcloud_merger::pointcloud1_sub_
   */
  void pointcloud1_callback_(sensor_msgs::PointCloud2 msg);
  /**
   * @brief ROS callback function for pointcloud2_topic (message type : sensor_msgs/PointCloud2)
   *
   * @param msg ROS message
   * @sa pointcloud_merger::pointcloud2_sub_
   */
  void pointcloud2_callback_(sensor_msgs::PointCloud2 msg);
  /**
   * @brief function for publishing pointcloud
   *
   */
  void publish_pointcloud_();
};

#endif  // POINTCLOUD_MERGER_H_INCLUDED