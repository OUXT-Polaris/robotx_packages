#ifndef BING_OBJECT_DETECTION_H_INCLUDED
#define BING_OBJECT_DETECTION_H_INCLUDED

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/saliency.hpp>

#include <vector>

#include <robotx_msgs/RegionOfInterest2DArray.h>

class bing_object_detection : public cv::saliency::ObjectnessBING {
 public:
  bing_object_detection();
  ~bing_object_detection();

 private:
  void _image_callback(const sensor_msgs::ImageConstPtr& msg);
  void _detect(cv::Mat image, std::vector<cv::Vec4i>& bboxs, std::vector<float>& objectness_vals);
  ros::NodeHandle _nh;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _image_sub;
  ros::Publisher _bbox_pub;
  int _max_num_bbox;
};

#endif  // BING_OBJECT_DETECTION_H_INCLUDED