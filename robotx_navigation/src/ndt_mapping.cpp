#include <ndt_mapping.h>

ndt_mapping::ndt_mapping()
    : params_(), sync_(odom_sub_, pointcloud_sub_, 10), pointcloud_buf_(2), odom_buf_(2) {
  ndt_.setTransformationEpsilon(0.01);
  ndt_.setStepSize(0.1);
  ndt_.setResolution(1.0);
  ndt_.setMaximumIterations(35);
  sync_.registerCallback(boost::bind(&ndt_mapping::callback_, this, _1, _2));
}

ndt_mapping::~ndt_mapping() {}

void ndt_mapping::callback_(const nav_msgs::OdometryConstPtr& odom_msg,
                            const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pointcloud_msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pointcloud);
  pointcloud_buf_.push_back(*pcl_pointcloud);
  odom_buf_.push_back(*odom_msg);
}