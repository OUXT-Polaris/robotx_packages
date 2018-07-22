#ifndef CNN_RESULT_VISUALIZER_H_INCLUDED
#define CNN_RESULT_VISUALIZER_H_INCLUDED

// headers in this package
#include <robotx_msgs/ObjectRegionOfInterestArray.h>

// headers in jsk_rviz_plugins
#include <jsk_rviz_plugins/PictogramArray.h>

// headers in ROS
#include <ros/ros.h>

class cnn_result_visualizer {
 public:
  struct parameters {
    std::string object_roi_topic;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/object_roi_topic", object_roi_topic,
                                     ros::this_node::getName() + "/object_roi");
    }
  };
  cnn_result_visualizer();
  ~cnn_result_visualizer();

 private:
  const parameters params_;
  ros::NodeHandle nh_;
  ros::Subscriber object_roi_sub_;
  ros::Publisher pictgram_array_pub_;
  void object_roi_callback(robotx_msgs::ObjectRegionOfInterestArray msg);
};
#endif  // CNN_RESULT_VISUALIZER_H_INCLUDED