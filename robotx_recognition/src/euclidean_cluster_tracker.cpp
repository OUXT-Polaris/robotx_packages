#include <euclidean_cluster_tracker.h>

euclidean_cluster_tracker::euclidean_cluster_tracker()
{
    pfilter_ = new particle_filter(3,1000);
}

euclidean_cluster_tracker::~euclidean_cluster_tracker()
{

}