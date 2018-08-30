/**
 * @mainpage twist_from_fix_velocity_node
 * @brief recive fix_velocity from utm_odometry_node and publish twist topic.
 *
 * This node is unde developing.
 * @author Masaya kataoka
 * @date 2018-06-10
 * @image html images/logo.jpg
 */

// headers in this package
#include <twist_calculator.h>

// headers for ros
#include <ros/ros.h>

/**
 * @brief entry point
 *
 * @param argc number of args
 * @param argv args
 * @return int aleays return 0;
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "twist_from_fix_velocity_node");
  twist_calculator calculator;
  ros::spin();
  return 0;
}
