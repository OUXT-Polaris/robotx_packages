#ifndef POINTCLOUD_MERGER_NODE_H_INCLUDED
#define POINTCLOUD_MERGER_NODE_H_INCLUDED

#include <nodelet/nodelet.h>
#include <pointcloud_merger.h>

namespace robotx_recognition_nodelet {
class pointcloud_merger_nodelet : public nodelet::Nodelet {
 public:
  pointcloud_merger_nodelet();
  ~pointcloud_merger_nodelet();
  virtual void onInit();

 private:
  pointcloud_merger merger;
};
}

#endif  // POINTCLOUD_MERGER_NODE_H_INCLUDED