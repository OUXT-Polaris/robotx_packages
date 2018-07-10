/**
 * @mainpage pass through filter ROS node.
 * @author Masaya Kataoka
 * @date 2018-06-10
 * @image images/logo.png
 */

/**
 * @brief pass through filter ROS node.
 *
 * @file passthrough_filter_node.cpp
 * @author Masaya Kataoka
 * @date 2018-06-10
 */

// headers in this package
#include <passthrough_filter.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "passthrough_filter_node");
  passthough_filter* filter_ptr = new passthough_filter();
  ros::spin();
  return 0;
}