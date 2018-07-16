#include <object_bbox_extractor_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace robotx_recognition_nodelet {
object_bbox_extractor_nodelet::object_bbox_extractor_nodelet() {}
object_bbox_extractor_nodelet::~object_bbox_extractor_nodelet() {}
void object_bbox_extractor_nodelet::onInit() {}
}

PLUGINLIB_EXPORT_CLASS(robotx_recognition_nodelet::object_bbox_extractor_nodelet, nodelet::Nodelet)