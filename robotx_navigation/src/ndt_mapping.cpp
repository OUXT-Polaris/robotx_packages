#include <ndt_mapping.h>

ndt_mapping::ndt_mapping() : params_(), sync_(odom_sub_, pointcloud_sub_, 10) {
  sync_.registerCallback(boost::bind(&ndt_mapping::callback_, this, _1, _2));
}

ndt_mapping::~ndt_mapping() {}

void ndt_mapping::callback_(const nav_msgs::OdometryConstPtr& odom_msg,
                            const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg) {}