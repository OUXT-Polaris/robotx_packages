/**
 * @brief implementation of passthrough_filter class
 *
 * @file passthrough_filter.cpp
 * @author Masaya Kataoka
 * @date 2018-06-10
 */

#include <passthrough_filter.h>

// headers in PCL
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

passthough_filter::passthough_filter() : param_(passthough_filter::parameters()) {
  pointcloud_filterd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(param_.output_cloud, 1);
  pointcloud_sub_ = nh_.subscribe(param_.input_cloud, 1, &passthough_filter::pointcloud_callback_, this);
}

void passthough_filter::pointcloud_callback_(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  if (param_.mode == remain) {
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    if (param_.min_x == 0 && param_.max_x == 0) {
    } else {
      pass.setFilterFieldName("x");
      pass.setFilterLimits(param_.min_x, param_.max_x);
    }
    if (param_.min_y == 0 && param_.max_y == 0) {
    } else {
      pass.setFilterFieldName("y");
      pass.setFilterLimits(param_.min_y, param_.max_y);
    }
    if (param_.min_z == 0 && param_.max_z == 0) {
    } else {
      pass.setFilterFieldName("z");
      pass.setFilterLimits(param_.min_x, param_.max_z);
    }
    pass.filter(*cloud_filtered);
  }
  if (param_.mode == remove) {
    for (pcl::PointCloud<pcl::PointXYZI>::iterator i = cloud->begin(); i != cloud->end(); i++) {
      bool valid_x;
      bool valid_y;
      bool valid_z;
      if (param_.min_x == 0 && param_.max_x == 0) {
        valid_x = true;
      } else {
        if (param_.min_x > (*i).x || param_.max_x < (*i).x) {
          valid_x = true;
        } else {
          valid_x = false;
        }
      }
      if (param_.min_y == 0 && param_.max_y == 0) {
        valid_y = true;
      } else {
        if (param_.min_y > (*i).y || param_.max_y < (*i).y) {
          valid_y = true;
        } else {
          valid_y = false;
        }
      }
      if (param_.min_z == 0 && param_.max_z == 0) {
        valid_z = true;
      } else {
        if (param_.min_z > (*i).z || param_.max_z < (*i).z) {
          valid_z = true;
        } else {
          valid_z = false;
        }
      }
      if (valid_x == true && valid_y == true && valid_z == true) {
        pcl::PointXYZI p((*i).intensity);
        p.x = (*i).x;
        p.y = (*i).y;
        p.z = (*i).z;
        cloud_filtered->push_back(p);
      }
    }
  }
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud_filtered, output_msg);
  output_msg.header = msg->header;
  pointcloud_filterd_pub_.publish(output_msg);
  return;
}