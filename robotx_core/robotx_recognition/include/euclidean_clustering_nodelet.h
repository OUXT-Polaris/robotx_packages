#ifndef EUCLIDEAN_CLUSTERING_NODELET_H_INCLUDED
#define EUCLIDEAN_CLUSTERING_NODELET_H_INCLUDED

#include <euclidean_clustering.h>
#include <euclidean_clustering.h>
#include <nodelet/nodelet.h>

namespace robotx_recognition_nodelet {
class euclidean_clustering_nodelet : public nodelet::Nodelet {
 public:
  euclidean_clustering_nodelet();
  ~euclidean_clustering_nodelet();
  virtual void onInit();

 private:
  euclidean_clustering clustering;
};
}

#endif  // EUCLIDEAN_CLUSTERING_NODELET_H_INCLUDED