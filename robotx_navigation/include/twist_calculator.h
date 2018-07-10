#ifndef TWIST_CALCULATOR_H_INCLUDED
#define TWIST_CALCULATOR_H_INCLUDED

/**
 * @brief definition of twist_calculator class
 *
 * @file twist_calculator.h
 * @author Masaya Kataoka
 * @date 2018-06-10
 */

// headers for ros
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * @brief calculate twist from fix_velocity
 *
 */
class twist_calculator {
 public:
  /**
   * @brief Construct a new twist calculator object
   *
   */
  twist_calculator();

 private:
  // callback functions
  /**
   * @brief ROS callback function for /fix_velocity topic
   *
   * @param msg ROS message(message type: geometry_msgs/Vector3Stamped)
   */
  void fix_velocity_callback(const geometry_msgs::Vector3Stamped msg);
  /**
   * @brief ROS callback function for /imu topic
   *
   * @param msg ROS message(message type: sensor_msgs/Imu)
   */
  void imu_callback(const sensor_msgs::Imu msg);
  // publishers and Subscribers
  /**
   * @brief ROS subscriber for /fix_velocity topic
   *
   */
  ros::Subscriber fix_velocity_sub;
  /**
   * @brief ROS subscriber for /imu topic
   *
   */
  ros::Subscriber imu_sub;
  /**
   * @brief ROS publisher for /twist topic
   *
   */
  ros::Publisher twist_pub;
  /**
   * @brief ROS NodeHandle
   *
   */
  ros::NodeHandle nh;
  // tf
  /**
   * @brief tf_buffer in tf2
   *
   */
  tf2_ros::Buffer *tf_buffer;
  /**
   * @brief TransformListene for tf2, read transform of ((fix_velocity_frame)
   * <-> (twist_frame),(fix_velocity_frame) <-> (imu_ftame)) frames.
   * @sa twist_calculator::fix_velocity_frame
   * @sa twist_calculator::twist_frame
   * @sa twist_calculator::imu_frame
   */
  tf2_ros::TransformListener *tf_listener;
  // parameters
  /**
   * @brief name of fix_velocity frame (default:world)
   *
   */
  std::string fix_velocity_frame;
  /**
   * @brief name of twist frame (default:base_link)
   *
   */
  std::string twist_frame;
  /**
   * @brief name of imu frame (default:imu)
   *
   */
  std::string imu_frame;
  // datas
  /**
   * @brief parameter for angular_velocity
   *
   */
  geometry_msgs::Vector3Stamped angular_velocity;
  /**
   * @brief parameter for linear_velocity
   *
   */
  geometry_msgs::Vector3Stamped linear_velocity;
};

#endif
