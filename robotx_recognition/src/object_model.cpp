#include <object_model.h>

//headers for ROS
#include <ros/ros.h>

//headers in pcl
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/shot_omp.h>

object_model::object_model(pcl::PointCloud<pcl::PointXYZ>::Ptr model,std::string object_name,std::string stl_file_path,std::string marker_mesh_path)
{
  object_name_ = object_name;
  stl_file_path_ = stl_file_path;
  marker_mesh_path_ = marker_mesh_path;
  model_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  model_keypoints_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  model_normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
  model_ = model;
  model_descriptors_ = pcl::PointCloud<pcl::SHOT352>::Ptr(new pcl::PointCloud<pcl::SHOT352>());
  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud(model_);
  norm_est.compute(*model_normals_);
  //
  //  Downsample Clouds to Extract keypoints
  //
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_filter;
  voxelgrid_filter.setInputCloud(model);
  voxelgrid_filter.setLeafSize(0.01f, 0.01f, 0.01f);
  voxelgrid_filter.filter(*model_keypoints_);
  //ROS_INFO_STREAM(model_->size() << " keypoints detected");
  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
  try
  {
    descr_est.setRadiusSearch(15);
    descr_est.setInputCloud(model_);
    descr_est.setInputNormals(model_normals_);
    descr_est.setSearchSurface(model_);
    descr_est.compute(*model_descriptors_);
    //ROS_INFO_STREAM("model description succeed.");
  }
  catch(...)
  {
    ROS_WARN_STREAM("model description failed.");
  }
}

object_model::object_model(pcl::PointCloud<pcl::PointXYZ>::Ptr model,std::string object_name)
{
  object_name_ = object_name;
  stl_file_path_ = "";
  marker_mesh_path_ = "";
  model_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  model_keypoints_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  model_normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
  model_ = model;
  model_descriptors_ = pcl::PointCloud<pcl::SHOT352>::Ptr(new pcl::PointCloud<pcl::SHOT352>());
  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setKSearch(10);
  norm_est.setInputCloud(model_);
  norm_est.compute(*model_normals_);
  //
  //  Downsample Clouds to Extract keypoints
  //
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_filter;
  voxelgrid_filter.setInputCloud(model);
  voxelgrid_filter.setLeafSize(0.01f, 0.01f, 0.01f);
  voxelgrid_filter.filter(*model_keypoints_);
  //ROS_INFO_STREAM(model_->size() << " keypoints detected");
  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
  try
  {
    descr_est.setRadiusSearch(15);
    descr_est.setInputCloud(model_);
    descr_est.setInputNormals(model_normals_);
    descr_est.setSearchSurface(model_);
    descr_est.compute(*model_descriptors_);
    //ROS_INFO_STREAM("model description succeed.");
  }
  catch(...)
  {
    ROS_WARN_STREAM("model description failed.");
  }
}

pcl::PointCloud<pcl::SHOT352>::Ptr object_model::get_model_descriptors()
{
  return model_descriptors_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr object_model::get_model_keypoints()
{
  return model_keypoints_;
}

pcl::PointCloud<pcl::Normal>::Ptr object_model::get_model_normals()
{
  return model_normals_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr object_model::get_model()
{
  return model_;
}

std::string object_model::get_name()
{
  return object_name_;
}

std::string object_model::get_stl_file_path()
{
  return stl_file_path_;
}

std::string object_model::get_marker_mesh_path()
{
  return marker_mesh_path_;
}

object_model::~object_model()
{

}
