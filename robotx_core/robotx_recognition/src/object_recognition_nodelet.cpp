#include <object_recognition_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace robotx_recognition_nodelet {
object_recognition_nodelet::object_recognition_nodelet() {}

object_recognition_nodelet::~object_recognition_nodelet() {}

void object_recognition_nodelet::onInit() {}
}

PLUGINLIB_EXPORT_CLASS(robotx_recognition_nodelet::object_recognition_nodelet, nodelet::Nodelet)