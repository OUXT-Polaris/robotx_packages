#ifndef PASSTHOUGH_FILTER_NODELET_H_INCLUDED
#define PASSTHOUGH_FILTER_NODELET_H_INCLUDED

#include <nodelet/nodelet.h>

namespace robotx_recognition_nodelet {
class passthrough_filter_nodelet : public nodelet::Nodelet {
 public:
  passthrough_filter_nodelet();
  ~passthrough_filter_nodelet();
  virtual void onInit();
};
}  // robotx_recognition

#endif  // PASSTHOUGH_FILTER_NODELET_H_INCLUDED