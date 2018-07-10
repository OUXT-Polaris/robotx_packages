#ifndef COSTMAP_CLEAR_REQUEST_SENDER_H_INCLUDED
#define COSTMAP_CLEAR_REQUEST_SENDER_H_INCLUDED

#include <ros/ros.h>
#include <std_srvs/Empty.h>

/**
 * @brief calling /move_base/clear_costmaps service
 *
 */
class costmap_clear_request_sender {
 public:
  /**
   * @brief parameters for costmap_clear_request_sender class
   *
   */
  struct parameters {
    /**
     * @brief request frequency
     *
     */
    double frequency;
    /**
     * @brief service name of clear costmaps
     *
     */
    std::string target_service_name;
    parameters() {
      ros::param::param<double>(ros::this_node::getName() + "/frequency", frequency, 2);
      ros::param::param<std::string>(ros::this_node::getName() + "/target_service_name", target_service_name,
                                     "/move_base/clear_costmaps");
    }
  };
  /**
   * @brief Construct a new costmap clear request sender object
   *
   */
  costmap_clear_request_sender();
  /**
   * @brief Destroy the costmap clear request sender object
   *
   */
  ~costmap_clear_request_sender();
  /**
   * @brief start loop for sending request
   *
   */
  void run();

 private:
  /**
   * @brief parameters for costmap_clear_request_sender class
   *
   */
  const parameters params_;
  /**
   * @brief ROS nodehandle
   *
   */
  ros::NodeHandle nh_;
  /**
   * @brief ROS Service Cleient
   *
   */
  ros::ServiceClient client_;
};
#endif  // COSTMAP_CLEAR_REQUEST_SENDER_H_INCLUDED