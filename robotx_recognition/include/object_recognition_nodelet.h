#ifndef OBJECT_RECOGNITION_NODELET_H_INCLUDED
#define OBJECT_RECOGNITION_NODELET_H_INCLUDED

#include <nodelet/nodelet.h>
#include <pcl_object_recognition.h>

namespace robotx_recognition_nodelet {
class object_recognition_nodelet : public nodelet::Nodelet {
 public:
  object_recognition_nodelet();
  ~object_recognition_nodelet();
  virtual void onInit();

 private:
  pcl_object_recognition object_recognition;
};
}

#endif  // OBJECT_RECOGNITION_NODELET_H_INCLUDED