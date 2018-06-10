#ifndef EUCLIDIAN_CLUSTER_TRACKER_H_INCLUDED
#define EUCLIDIAN_CLUSTER_TRACKER_H_INCLUDED

#include <ros/ros.h>

#include "particle_filter.h"
//#include <robotx_msgs/EuclideanClusters.h>

class euclidean_cluster_tracker
{
public:
    euclidean_cluster_tracker();
    ~euclidean_cluster_tracker();
private:
    //void euclidean_cluster_callback_(const robotx_msgs::EuclideanClustersConstPtr& msg);
    ros::NodeHandle nh_;
    //particle_filter* pfilter_;
};

#endif  //EUCLIDIAN_CLUSTER_TRACKER_H_INCLUDED