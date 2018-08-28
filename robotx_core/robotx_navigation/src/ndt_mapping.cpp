#include <ndt_mapping.h>

ndt_mapping::ndt_mapping()
    : params_(),
      odom_sub_(nh_, params_.odom_topic, 1),
      pointcloud_sub_(nh_, params_.pointcloud_topic, 1),
      sync_(odom_sub_, pointcloud_sub_, 10),
      pointcloud_buf_(2),
      odom_buf_(2),
      map_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>) {
  ndt_.setTransformationEpsilon(0.01);
  ndt_.setStepSize(0.1);
  ndt_.setResolution(1.0);
  ndt_.setMaximumIterations(35);
  approximate_voxel_filter_.setLeafSize(params_.map_resolution, params_.map_resolution,
                                        params_.map_resolution);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/map", 1);
  sync_.registerCallback(boost::bind(&ndt_mapping::callback_, this, _1, _2));
}

ndt_mapping::~ndt_mapping() {}

void ndt_mapping::callback_(const nav_msgs::OdometryConstPtr &odom_msg,
                            const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pointcloud_msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pointcloud);
  pointcloud_buf_.push_back(pcl_pointcloud);
  odom_buf_.push_back(*odom_msg);
  if (odom_buf_.size() == 2 && pointcloud_buf_.size() == 2) {
    ndt_.setInputTarget(pointcloud_buf_[0]);
    ndt_.setInputSource(pointcloud_buf_[1]);
    double r0, p0, y0;
    double r1, p1, y1;
    get_rpy_(odom_buf_[0].pose.pose.orientation, r0, p0, y0);
    get_rpy_(odom_buf_[1].pose.pose.orientation, r1, p1, y1);
    Eigen::AngleAxisf odom_rot(y1 - y0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f odom_trans(odom_buf_[1].pose.pose.position.x - odom_buf_[0].pose.pose.position.x,
                                    odom_buf_[1].pose.pose.position.y - odom_buf_[0].pose.pose.position.y, 0);
    Eigen::Matrix4f init_guess = (odom_trans * odom_rot).matrix();
    ndt_.align(*map_pointcloud_, init_guess);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pointcloud_buf_[0], *transformed_cloud, ndt_.getFinalTransformation());
    *map_pointcloud_ += *transformed_cloud;
    approximate_voxel_filter_.setInputCloud(map_pointcloud_);
    approximate_voxel_filter_.filter(*map_pointcloud_);
    pointcloud_buf_[1] = map_pointcloud_;
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*map_pointcloud_, output_msg);
    pointcloud_pub_.publish(output_msg);
  }
}

void ndt_mapping::get_rpy_(geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw) {
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}