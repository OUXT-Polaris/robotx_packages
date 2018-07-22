#include <cnn_result_visualizer.h>

cnn_result_visualizer::cnn_result_visualizer() : params_() {
  object_roi_sub_ =
      nh_.subscribe(params_.object_roi_topic, 1, &cnn_result_visualizer::object_roi_callback, this);
  pictgram_array_pub_ =
      nh_.advertise<jsk_rviz_plugins::PictogramArray>(ros::this_node::getName() + "/pictgram", 1);
}

cnn_result_visualizer::~cnn_result_visualizer() {}

void cnn_result_visualizer::object_roi_callback(robotx_msgs::ObjectRegionOfInterestArray msg) {
  jsk_rviz_plugins::PictogramArray pictgram_msg;
  jsk_rviz_plugins::Pictogram pict;
}