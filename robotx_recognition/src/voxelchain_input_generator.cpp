//headers in this package
#include <voxelchain_input_generator.h>


voxelchain_input_generator::voxelchain_input_generator()
{
  euclidean_clusters_sub_ = nh_.subscribe(ros::this_node::getName()+"/clusters", 1,
    &voxelchain_input_generator::euclidean_clusters_callback, this);
}

voxelchain_input_generator::~voxelchain_input_generator()
{

}

void voxelchain_input_generator::euclidean_clusters_callback(robotx_msgs::EuclideanClusters msg)
{

}
