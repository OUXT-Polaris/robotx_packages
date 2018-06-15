#include <pointcloud_merger.h>

pointcloud_merger::pointcloud_merger() : params_(),tf_listener_(tf_buffer_)
{
    pointcloud1_sub_ = nh_.subscribe(params_.pointcloud1_topic, 1, &pointcloud_merger::pointcloud1_callback_, this);
    pointcloud2_sub_ = nh_.subscribe(params_.pointcloud1_topic, 1, &pointcloud_merger::pointcloud2_callback_, this);
}

pointcloud_merger::~pointcloud_merger()
{

}

void pointcloud_merger::pointcloud1_callback_(sensor_msgs::PointCloud2 msg)
{
    
}

void pointcloud_merger::pointcloud2_callback_(sensor_msgs::PointCloud2 msg)
{

}

void pointcloud_merger::publish_pointcloud_()
{
}