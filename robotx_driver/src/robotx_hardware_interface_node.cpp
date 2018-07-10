/**
 * @mainpage ROS node for hardware/simulation interface
 * @author Masaya Kataoka
 * @date 2018-06-11
 * @image images/logo.jpg
 */

/**
 * @brief ROS node for hardware/simulation interface
 *
 * @file robotx_hardware_interface_node.cpp
 * @author Masaya Kataoka
 * @date 2018-06-11
 */

// headers in ROS
#include <robotx_hardware_interface.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robotx_hardware_interface_node");
  robotx_hardware_interface *interface = new robotx_hardware_interface();
  ros::spin();
  return 0;
}