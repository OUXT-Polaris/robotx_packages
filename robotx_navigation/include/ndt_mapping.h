#ifndef NDT_MAPPING_H_INCLUDED
#define NDT_MAPPING_H_INCLUDED

#include <ros/ros.h>

class ndt_mapping {
 public:
  struct parameters {
    std::string pointcloud_topic;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/pointcloud_topic", pointcloud_topic,
                                     ros::this_node::getName() + "/input_pointcloud");
    }
  };
  ndt_mapping();
  ~ndt_mapping();

 private:
  const parameters params_;
  ros::NodeHandle nh_;
};
#endif  // NDT_MAPPING_H_INCLUDED