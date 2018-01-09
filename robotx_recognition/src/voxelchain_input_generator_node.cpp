//headers in this package
#include <voxelchain_input_generator.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "voxelchain_input_generator_node");
  voxelchain_input_generator generator;
  ros::spin();
  return 0;
}
