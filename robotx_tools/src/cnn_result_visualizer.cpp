#include <cnn_result_visualizer.h>

cnn_result_visualizer::cnn_result_visualizer() : params_() {
  object_roi_sub_ =
      nh_.subscribe(params_.object_roi_topic, 1, &cnn_result_visualizer::object_roi_callback, this);
  pictgram_array_pub_ =
      nh_.advertise<jsk_rviz_plugins::PictogramArray>(ros::this_node::getName() + "/pictgram", 1);
}

cnn_result_visualizer::~cnn_result_visualizer() {}

void cnn_result_visualizer::object_roi_callback(
    const robotx_msgs::ObjectRegionOfInterestArray::ConstPtr msg) {
  jsk_rviz_plugins::PictogramArray pictgram_msg;
  for (auto msg_itr = msg->object_rois.begin(); msg_itr != msg->object_rois.end(); msg_itr++) {
    if (msg_itr == msg->object_rois.begin()) {
      pictgram_msg.header = msg_itr->roi_3d.header;
    }
    jsk_rviz_plugins::Pictogram single_pict;
    single_pict.header = msg_itr->roi_3d.header;
    single_pict.pose = msg_itr->roi_3d.pose;
    single_pict.pose.position.z = msg_itr->roi_3d.pose.position.z + params_.z_offset;
    single_pict.action = single_pict.ADD;
    single_pict.mode = single_pict.STRING_MODE;
    switch (msg_itr->object_type.ID) {
      // msg_itr->object_type.OTHER
      case 0:
        single_pict.character = "OTHER";
        break;
      // msg_itr->object_type.GREEN_BUOY
      case 1:
        single_pict.character = "GREEN_BUOY";
        break;
      // msg_itr->object_type.RED_BUOY
      case 2:
        single_pict.character = "RED_BUOY";
        break;
      // msg_itr->object_type.WHITE_BUOY
      case 3:
        single_pict.character = "WHITE_BUOY";
        break;
    }
    single_pict.color.r = 0;
    single_pict.color.g = 0.5;
    single_pict.color.b = 1.0;
    single_pict.color.a = 1.0;
    single_pict.size = 1.0;
    single_pict.ttl = 0.5;
    single_pict.speed = 0.0;
    pictgram_msg.pictograms.push_back(single_pict);
  }
  pictgram_array_pub_.publish(pictgram_msg);
}