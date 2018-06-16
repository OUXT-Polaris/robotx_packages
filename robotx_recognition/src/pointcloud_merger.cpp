#include <pointcloud_merger.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

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
    pointcloud1_msg_ = msg;
    publish_pointcloud_();
    return;
}

void pointcloud_merger::pointcloud2_callback_(sensor_msgs::PointCloud2 msg)
{
    pointcloud2_msg_ = msg;
    return;
}

void pointcloud_merger::publish_pointcloud_()
{
    
    if(pointcloud1_msg_.header.frame_id != pointcloud2_msg_.header.frame_id)
    {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_.lookupTransform(pointcloud2_msg_.header.frame_id, pointcloud1_msg_.header.frame_id,ros::Time(0));
        tf2::doTransform(pointcloud2_msg_, pointcloud2_msg_, transform_stamped);
    }
    return;
}