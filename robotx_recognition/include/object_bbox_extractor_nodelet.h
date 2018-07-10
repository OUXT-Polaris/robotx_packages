#ifndef EUCLIDEAN_CLUSTERING_NODELET_H_INCLUDED
#define EUCLIDEAN_CLUSTERING_NODELET_H_INCLUDED

#include <nodelet/nodelet.h>
#include <object_bbox_extractor.h>
#include <object_bbox_extractor.h>

namespace robotx_recognition_nodelet {
class object_bbox_extractor_nodelet : public nodelet::Nodelet {
 public:
  object_bbox_extractor_nodelet();
  ~object_bbox_extractor_nodelet();
  virtual void onInit();

 private:
  object_bbox_extractor clustering;
};
}

#endif  // EUCLIDEAN_CLUSTERING_NODELET_H_INCLUDED