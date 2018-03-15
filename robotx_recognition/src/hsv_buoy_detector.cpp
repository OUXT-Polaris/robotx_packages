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
      threshold_param_itr->second["min_v"],threshold_param_itr->second["max_v"],
      threshold_param_itr->second["min_area"],threshold_param_itr->second["max_area"]);
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
  cv::Mat hsv_image;
  cv::cvtColor(src_image, hsv_image, CV_RGB2HSV, 3);
  masked_images_ = std::vector<cv::Mat>(threashold_params.size());
  //generate masked images for all objects and calculate contours
  for(int i = 0;i < masked_images_.size();i++)
  {
    double min_h,max_h,min_s,max_s,min_v,max_v;
    threashold_params[i].get_threshold(min_h,max_h,min_s,max_s,min_v,max_v);
    cv::inRange(hsv_image, cv::Scalar(min_h, min_s, min_v, 0) , cv::Scalar(max_h, max_s, max_v, 0), masked_images_[i]);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(masked_images_[i],contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    double min_area,max_area;
    threashold_params[i].get_area_threshold(min_area,max_area);
    //check contour size for ALL contours
    for(std::vector<std::vector<cv::Point> >::const_iterator itr = contours.begin(); itr != contours.end(); ++itr)
    {
      std::vector<cv::Point> approx;
      approxPolyDP(*itr, approx, 5, true);
      double area = contourArea(approx);
      if(max_area != 0)
      {
        if(max_area >= area && area >= min_area)
        {
          cv::boundingRect(approx);
        }
      }
      else
      {
        if(area >= min_area)
        {
          cv::boundingRect(approx);
        }
      }
    }
  }
}
