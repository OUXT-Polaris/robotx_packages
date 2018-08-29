#include <passthrough_filter_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace robotx_recognition_nodelet {
passthrough_filter_nodelet::passthrough_filter_nodelet() {}
passthrough_filter_nodelet::~passthrough_filter_nodelet() {}
void passthrough_filter_nodelet::onInit() {}
}  // robotx_recognition

PLUGINLIB_EXPORT_CLASS(robotx_recognition_nodelet::passthrough_filter_nodelet, nodelet::Nodelet)