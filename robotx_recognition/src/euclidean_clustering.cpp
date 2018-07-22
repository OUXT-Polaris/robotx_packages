// headers in this package
#include <euclidean_clustering.h>

// headers in boost
#include <boost/bind.hpp>
#include <boost/function.hpp>

// headers in STL
#include <random>

euclidean_clustering::euclidean_clustering()
    : conditional_euclidian_clustering_params_(), euclidian_clustering_params_(), bbox_params_() {
  marker_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(ros::this_node::getName() + "/bbox", 1);
  ros::param::param<int>(ros::this_node::getName() + "/min_cluster_size", min_cluster_size_, 10);
  ros::param::param<int>(ros::this_node::getName() + "/max_cluster_size", max_cluster_size_, 1000);
  ros::param::param<bool>(ros::this_node::getName() + "/donwsample/enable", donwsample_, true);
  ros::param::param<double>(ros::this_node::getName() + "/donwsample/voxel_grid/leaf_size/x", leaf_size_x,
                            0.01);
  ros::param::param<double>(ros::this_node::getName() + "/donwsample/voxel_grid/leaf_size/y", leaf_size_y,
                            0.01);
  ros::param::param<double>(ros::this_node::getName() + "/donwsample/voxel_grid/leaf_size/z", leaf_size_z,
                            0.01);
  ros::param::param<std::string>(ros::this_node::getName() + "/input_cloud", input_cloud_,
                                 ros::this_node::getName() + "/input_cloud");
  ros::param::param<int>(ros::this_node::getName() + "/clustering_method", clustering_method_,
                         CONDITIONAL_EUCLIDIAN_CLUSTERING);
  pointcloud_sub_ = nh_.subscribe(input_cloud_, 5, &euclidean_clustering::poincloud_callback, this);
}

euclidean_clustering::~euclidean_clustering() {}

void euclidean_clustering::poincloud_callback(sensor_msgs::PointCloud2 msg) { make_cluster(msg); }

bool euclidean_clustering::custom_region_growing_function(const pcl::PointXYZINormal& point_a,
                                                          const pcl::PointXYZINormal& point_b,
                                                          float squared_distance) {
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(),
                                    point_b_normal = point_b.getNormalVector3fMap();
  if (squared_distance < 10000) {
    if (fabs(point_a.intensity - point_b.intensity) < 8.0f) return (true);
    if (fabs(point_a_normal.dot(point_b_normal)) < 0.06) return (true);
  } else {
    if (fabs(point_a.intensity - point_b.intensity) < 3.0f) return (true);
  }
  return (false);
}

bool euclidean_clustering::enforce_intensity_similarity(const pcl::PointXYZINormal& point_a,
                                                        const pcl::PointXYZINormal& point_b,
                                                        float squared_distance) {
  if (fabs(point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  else
    return (false);
}

void euclidean_clustering::make_cluster(sensor_msgs::PointCloud2 msg) {
  if (clustering_method_ == CONDITIONAL_EUCLIDIAN_CLUSTERING) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pointcloud);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_cloud_with_normals(
        new pcl::PointCloud<pcl::PointXYZINormal>);
    if (donwsample_) {
      // Downsample pointcloud
      pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
      voxel_grid.setInputCloud(pcl_pointcloud);
      voxel_grid.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
      voxel_grid.setDownsampleAllData(true);
      voxel_grid.filter(*pcl_pointcloud);
    }
    // Set up a Normal Estimation class and merge data in cloud_with_normals
    pcl::copyPointCloud(*pcl_pointcloud, *pcl_cloud_with_normals);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> normal_estimation;
    normal_estimation.setInputCloud(pcl_pointcloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZI>);
    normal_estimation.setSearchMethod(search_tree);
    normal_estimation.setRadiusSearch(conditional_euclidian_clustering_params_.radius_search);
    normal_estimation.compute(*pcl_cloud_with_normals);

    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> conditional_euclidean_clustering(true);
    conditional_euclidean_clustering.setInputCloud(pcl_cloud_with_normals);

    conditional_euclidean_clustering.setConditionFunction(
        &euclidean_clustering::custom_region_growing_function);
    // conditional_euclidean_clustering.setConditionFunction(&euclidean_clustering::enforce_intensity_similarity);
    conditional_euclidean_clustering.setClusterTolerance(
        conditional_euclidian_clustering_params_.cluster_tolerance);
    conditional_euclidean_clustering.setMinClusterSize(min_cluster_size_);
    conditional_euclidean_clustering.setMaxClusterSize(max_cluster_size_);
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters),
        large_clusters(new pcl::IndicesClusters);
    ;
    conditional_euclidean_clustering.segment(*clusters);
    conditional_euclidean_clustering.getRemovedClusters(small_clusters, large_clusters);
    // Generate marker message
    jsk_recognition_msgs::BoundingBoxArray markers;
    markers.header = msg.header;
    for (int i = 0; i < clusters->size(); ++i) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_clusterd(new pcl::PointCloud<pcl::PointXYZI>);
      pcl_pointcloud_clusterd->resize((*clusters)[i].indices.size());
      for (int j = 0; j < (*clusters)[i].indices.size(); ++j) {
        pcl_pointcloud_clusterd->points[j] = pcl_pointcloud->points[(*clusters)[i].indices[j]];
      }
      sensor_msgs::PointCloud2 pointcloud_clusterd_msg;
      pcl::toROSMsg(*pcl_pointcloud_clusterd, pointcloud_clusterd_msg);
      pointcloud_clusterd_msg.header = msg.header;
      // calculating bounding box
      std::vector<float> moment_of_inertia;
      std::vector<float> eccentricity;
      pcl::PointXYZI min_point, max_point;
      pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
      feature_extractor.setInputCloud(pcl_pointcloud_clusterd);
      feature_extractor.compute();
      feature_extractor.getMomentOfInertia(moment_of_inertia);
      feature_extractor.getEccentricity(eccentricity);
      feature_extractor.getAABB(min_point, max_point);
      // generate marker
      jsk_recognition_msgs::BoundingBox marker;
      marker.header = msg.header;
      marker.dimensions.x = std::fabs(max_point.x - min_point.x) + bbox_params_.inflation_size;
      marker.dimensions.y = std::fabs(max_point.y - min_point.y) + bbox_params_.inflation_size;
      marker.dimensions.z = std::fabs(max_point.z - min_point.z);
      marker.pose.position.x = (max_point.x + min_point.x) / 2;
      marker.pose.position.y = (max_point.y + min_point.y) / 2;
      marker.pose.position.z = (max_point.z + min_point.z) / 2;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;
      markers.boxes.push_back(marker);
    }
    marker_pub_.publish(markers);
  }
  if (clustering_method_ == EUCLIDIAN_CLUSTER_EXTRACTION) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pointcloud);
    if (donwsample_) {
      // Downsample pointcloud
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setInputCloud(pcl_pointcloud);
      voxel_grid.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
      voxel_grid.setDownsampleAllData(true);
      voxel_grid.filter(*pcl_pointcloud);
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pcl_pointcloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(euclidian_clustering_params_.cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl_pointcloud);
    ec.extract(cluster_indices);

    jsk_recognition_msgs::BoundingBoxArray markers;
    markers.header = msg.header;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_cluster->points.push_back(pcl_pointcloud->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      // calculating bounding box
      std::vector<float> moment_of_inertia;
      std::vector<float> eccentricity;
      pcl::PointXYZ min_point, max_point;
      pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
      feature_extractor.setInputCloud(cloud_cluster);
      feature_extractor.compute();
      feature_extractor.getMomentOfInertia(moment_of_inertia);
      feature_extractor.getEccentricity(eccentricity);
      feature_extractor.getAABB(min_point, max_point);
      // generate marker
      jsk_recognition_msgs::BoundingBox marker;
      marker.header = msg.header;
      marker.dimensions.x = std::fabs(max_point.x - min_point.x) + bbox_params_.inflation_size;
      marker.dimensions.y = std::fabs(max_point.y - min_point.y) + bbox_params_.inflation_size;
      marker.dimensions.z = std::fabs(max_point.z - min_point.z);
      marker.pose.position.x = (max_point.x + min_point.x) / 2;
      marker.pose.position.y = (max_point.y + min_point.y) / 2;
      marker.pose.position.z = (max_point.z + min_point.z) / 2;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;
      markers.boxes.push_back(marker);
    }
    marker_pub_.publish(markers);
  }
}
