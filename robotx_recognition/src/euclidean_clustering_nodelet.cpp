#include <euclidean_clustering_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace robotx_recognition_nodelet {
euclidean_clustering_nodelet::euclidean_clustering_nodelet() {}
euclidean_clustering_nodelet::~euclidean_clustering_nodelet() {}
void euclidean_clustering_nodelet::onInit() {}
}

PLUGINLIB_EXPORT_CLASS(robotx_recognition_nodelet::euclidean_clustering_nodelet, nodelet::Nodelet)