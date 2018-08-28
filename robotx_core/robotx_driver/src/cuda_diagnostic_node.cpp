/**
 * @mainpage cuda_diagnostic
 * ROS node for cuda diagnostic
 * @author Masaya Kataoka
 * @date 2018.06.28
 * @image html images/logo.jpg
 */

// headers in this package
#include <cuda_diagnostic.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cuda_diagnostic_node");
  cuda_diagnostic cuda_diagnostic_updater;
  cuda_diagnostic_updater.run();
  ros::spin();
  return 0;
}