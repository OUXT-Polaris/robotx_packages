#ifndef EUCLIDIAN_CLUSTER_TRACKER_H_INCLUDED
#define EUCLIDIAN_CLUSTER_TRACKER_H_INCLUDED

#include "particle_filter.h"

class euclidean_cluster_tracker
{
public:
    euclidean_cluster_tracker();
    ~euclidean_cluster_tracker();
private:
    particle_filter* pfilter_;
};

#endif  //EUCLIDIAN_CLUSTER_TRACKER_H_INCLUDED