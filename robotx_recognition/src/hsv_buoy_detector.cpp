//headers in this package
#include <hsv_buoy_detector.h>

//headers in opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

hsv_buoy_detector::hsv_buoy_detector():it_(nh_)
{
  XmlRpc::XmlRpcValue parameters;
  nh_.getParam(ros::this_node::getName(), parameters);
  for(auto threshold_param_itr = parameters.begin(); threshold_param_itr != parameters.end(); ++threshold_param_itr)
  {
    hsv_threshold threashold_param(threshold_param_itr->first,
      threshold_param_itr->second["min_h"],threshold_param_itr->second["max_h"],
      threshold_param_itr->second["min_s"],threshold_param_itr->second["max_s"],
      threshold_param_itr->second["min_v"],threshold_param_itr->second["max_v"]);
    threashold_params.push_back(threashold_param);
  }
  image_sub_ = it_.subscribe("/wam_v/front_camera/image_raw", 1, &hsv_buoy_detector::image_callback, this);
}

hsv_buoy_detector::~hsv_buoy_detector()
{

}

void hsv_buoy_detector::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat src_image = cv_ptr->image;
}
