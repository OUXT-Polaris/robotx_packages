// headers in this package
#include <pointcloud_merger.h>

// headers in ROS
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// headers in STL
#include <mutex>

// headers in PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

pointcloud_merger::pointcloud_merger() : params_(), tf_listener_(tf_buffer_) {
  output_pointcloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/merged_points", 1);
  pointcloud1_sub_ =
      nh_.subscribe(params_.pointcloud1_topic, 1, &pointcloud_merger::pointcloud1_callback_, this);
  pointcloud2_sub_ =
      nh_.subscribe(params_.pointcloud2_topic, 1, &pointcloud_merger::pointcloud2_callback_, this);
}

pointcloud_merger::~pointcloud_merger() {}

void pointcloud_merger::pointcloud1_callback_(sensor_msgs::PointCloud2 msg) {
  pointcloud1_msg_ = msg;
  publish_pointcloud_();
  return;
}

void pointcloud_merger::pointcloud2_callback_(sensor_msgs::PointCloud2 msg) {
  pointcloud2_msg_ = msg;
  return;
}

void pointcloud_merger::publish_pointcloud_() {
  std::mutex mutex;
  mutex.lock();
  if (pointcloud1_msg_.header.frame_id == "" || pointcloud2_msg_.header.frame_id == "") {
    return;
  }
  if (pointcloud1_msg_.header.frame_id != pointcloud2_msg_.header.frame_id) {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_.lookupTransform(pointcloud1_msg_.header.frame_id,
                                                   pointcloud2_msg_.header.frame_id, ros::Time(0));
    tf2::doTransform(pointcloud2_msg_, pointcloud2_msg_, transform_stamped);
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(pointcloud1_msg_, *pcl_cloud1);
  pcl::fromROSMsg(pointcloud2_msg_, *pcl_cloud2);
  *pcl_output_cloud = *pcl_cloud1 + *pcl_cloud2;
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*pcl_output_cloud, output_msg);
  output_pointcloud_pub_.publish(output_msg);
  mutex.unlock();
  return;
}