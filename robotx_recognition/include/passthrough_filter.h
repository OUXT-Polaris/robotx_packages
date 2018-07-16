/**
 * @brief definition of passthrough filter class
 *
 * @file passthrough_filter.h
 * @author Masaya Kataoka
 * @date 2018-06-10
 */

#ifndef PASSTHROUGH_FILTER_H_INCLUDED
#define PASSTHROUGH_FILTER_H_INCLUDED

// headers for ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * @brief passthrought filter class
 *
 */
class passthough_filter {
 public:
  /**
   * @brief parameters for passthough_filter class
   *
   */
  struct parameters {
    int mode;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
    std::string input_cloud;
    std::string output_cloud;
    /**
     * @brief default parameters
     *
     */
    parameters() {
      // if you want run as remove mode, set mode = 1
      ros::param::param<int>(ros::this_node::getName() + "/mode", mode, remain);
      ros::param::param<double>(ros::this_node::getName() + "/min_x", min_x, 0.0);
      ros::param::param<double>(ros::this_node::getName() + "/min_y", min_y, 0.0);
      ros::param::param<double>(ros::this_node::getName() + "/min_z", min_z, 0.0);
      ros::param::param<double>(ros::this_node::getName() + "/max_x", max_x, 0.0);
      ros::param::param<double>(ros::this_node::getName() + "/max_y", max_y, 0.0);
      ros::param::param<double>(ros::this_node::getName() + "/max_z", max_z, 0.0);
      ros::param::param<std::string>(ros::this_node::getName() + "/input_cloud", input_cloud,
                                     ros::this_node::getName() + "/input_cloud");
      ros::param::param<std::string>(ros::this_node::getName() + "/output_cloud", output_cloud,
                                     ros::this_node::getName() + "/output_cloud");
    }
  };
  /**
   * @brief Construct a new passthough filter object
   *
   */
  passthough_filter();
  /**
   * @brief Destroy the passthough filter object
   *
   */
  ~passthough_filter();
  const parameters param_;

 private:
  /**
   * @brief operating mode
   *
   * remain : remain pointclouds in the box
   * remove : remove pointclouds in the box
   */
  enum modes { remain = 0, remove = 1 };
  /**
   * @brief RPS nodehandle
   *
   */
  ros::NodeHandle nh_;
  /**
   * @brief ROS subscriber for (~/input_cloud ROS param) topic
   *
   */
  ros::Subscriber pointcloud_sub_;
  /**
   * @brief ROS publisher for (~/output_cloud ROS param) topic
   *
   */
  ros::Publisher pointcloud_filterd_pub_;
  /**
   * @brief ROS callback function for (~/input_cloud ROS param) topic
   *
   * @param msg ROS message (message type : sensor_msgs/PointCloud2)
   * @sa passthough_filter::pointcloud_sub_
   */
  void pointcloud_callback_(const sensor_msgs::PointCloud2ConstPtr& msg);
};

#endif  // PASSTHROUGH_FILTER_H_INCLUDED