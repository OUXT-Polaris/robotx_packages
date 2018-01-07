//headers in this package
#include <euclidean_clustering.h>

//headers in boost
#include <boost/function.hpp>
#include <boost/bind.hpp>

euclidean_clustering::euclidean_clustering()
{
  ros::param::param<int>(ros::this_node::getName()+"/min_cluster_size", min_cluster_size_, 10);
  ros::param::param<int>(ros::this_node::getName()+"/max_cluster_size", max_cluster_size_, 1000);
  ros::param::param<double>(ros::this_node::getName()+"/cluster_tolerance", cluster_tolerance_, 1);
  ros::param::param<double>(ros::this_node::getName()+"/cluster_tolerance", radius_search_, 0.5);
  ros::param::param<double>(ros::this_node::getName()+"/voxel_grid/leaf_size/x", leaf_size_x, 0.01);
  ros::param::param<double>(ros::this_node::getName()+"/voxel_grid/leaf_size/y", leaf_size_y, 0.01);
  ros::param::param<double>(ros::this_node::getName()+"/voxel_grid/leaf_size/z", leaf_size_z, 0.01);
  ros::param::param<std::string>(ros::this_node::getName()+"/input_cloud", input_cloud_, ros::this_node::getName()+"/input_cloud");
  pointcloud_sub_ = nh_.subscribe(input_cloud_, 5, &euclidean_clustering::poincloud_callback, this);
}

euclidean_clustering::~euclidean_clustering()
{

}

void euclidean_clustering::poincloud_callback(sensor_msgs::PointCloud2 msg)
{
  pointcloud_ = msg;
  make_cluster();
}

bool custom_region_growing_function(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
  if (squared_distance < 10000)
  {
    if (fabs (point_a.intensity - point_b.intensity) < 8.0f)
      return (true);
    if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  }
  else
  {
    if (fabs (point_a.intensity - point_b.intensity) < 3.0f)
      return (true);
  }
  return (false);
}

void euclidean_clustering::make_cluster()
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(pointcloud_,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2,*pcl_pointcloud);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
  //Downsample pointcloud
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setInputCloud(pcl_pointcloud);
  voxel_grid.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
  voxel_grid.setDownsampleAllData(true);
  voxel_grid.filter(*pcl_pointcloud);
  //Set up a Normal Estimation class and merge data in cloud_with_normals
  pcl::copyPointCloud(*pcl_pointcloud, *pcl_cloud_with_normals);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> normal_estimation;
  normal_estimation.setInputCloud(pcl_pointcloud);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZI>);
  normal_estimation.setSearchMethod(search_tree);
  normal_estimation.setRadiusSearch(radius_search_);
  normal_estimation.compute(*pcl_cloud_with_normals);
  //Set up a Conditional Euclidean Clustering class
  pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> conditional_euclidean_clustering(true);
  conditional_euclidean_clustering.setInputCloud(pcl_cloud_with_normals);
  conditional_euclidean_clustering.setConditionFunction(custom_region_growing_function);
  conditional_euclidean_clustering.setClusterTolerance(cluster_tolerance_);
  conditional_euclidean_clustering.setMinClusterSize(min_cluster_size_);
  conditional_euclidean_clustering.setMaxClusterSize(max_cluster_size_);
  pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters),small_clusters(new pcl::IndicesClusters),large_clusters(new pcl::IndicesClusters);;
  conditional_euclidean_clustering.segment(*clusters);
  conditional_euclidean_clustering.getRemovedClusters(small_clusters,large_clusters);
  // Using the intensity channel for lazy visualization of the output
  for (int i = 0; i < small_clusters->size(); ++i)
  {
    for (int j = 0; j < (*small_clusters)[i].indices.size(); ++j)
    {
      pcl_pointcloud->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
    }
  }
  for (int i = 0; i < large_clusters->size(); ++i)
  {
    for (int j = 0; j < (*large_clusters)[i].indices.size(); ++j)
    {
      pcl_pointcloud->points[(*large_clusters)[i].indices[j]].intensity = +10.0;
    }
  }
  for (int i = 0; i < clusters->size(); ++i)
  {
    int label = rand () % 8;
    for (int j = 0; j < (*clusters)[i].indices.size(); ++j)
    {
      pcl_pointcloud->points[(*clusters)[i].indices[j]].intensity = label;
    }
  }
  ROS_INFO_STREAM(clusters->size() << " clusters are found!!");
}
