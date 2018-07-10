#include <pluginlib/class_list_macros.h>
#include <pointcloud_merger_nodelet.h>

namespace robotx_recognition_nodelet {
pointcloud_merger_nodelet::pointcloud_merger_nodelet() {}

pointcloud_merger_nodelet::~pointcloud_merger_nodelet() {}

void pointcloud_merger_nodelet::onInit() {}
}

PLUGINLIB_EXPORT_CLASS(robotx_recognition_nodelet::pointcloud_merger_nodelet, nodelet::Nodelet)