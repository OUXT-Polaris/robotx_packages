// headers in this package
#include <hsv_buoy_detector.h>

// headers in opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

hsv_buoy_detector::hsv_buoy_detector() : it_(nh_) {
  roi_array_pub_ =
      nh_.advertise<robotx_msgs::RegionOfInterest2DArray>(ros::this_node::getName() + "/result", 10);
  XmlRpc::XmlRpcValue parameters;
  nh_.getParam(ros::this_node::getName(), parameters);
  for (auto threshold_param_itr = parameters.begin(); threshold_param_itr != parameters.end();
       ++threshold_param_itr) {
    hsv_threshold threashold_param(threshold_param_itr->first, threshold_param_itr->second["min_h"],
                                   threshold_param_itr->second["max_h"], threshold_param_itr->second["min_s"],
                                   threshold_param_itr->second["max_s"], threshold_param_itr->second["min_v"],
                                   threshold_param_itr->second["max_v"],
                                   threshold_param_itr->second["min_area"],
                                   threshold_param_itr->second["max_area"]);
    threashold_params.push_back(threashold_param);
  }
  image_sub_ =
      it_.subscribe(ros::this_node::getName() + "/image_raw", 1, &hsv_buoy_detector::image_callback, this);
}

hsv_buoy_detector::~hsv_buoy_detector() {}

void hsv_buoy_detector::image_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat src_image = cv_ptr->image;
  cv::Mat hsv_image;
  cv::cvtColor(src_image, hsv_image, CV_RGB2HSV, 3);
  masked_images_ = std::vector<cv::Mat>(threashold_params.size());
  // robotx_msgs::Objects2D objects2d_msg;
  robotx_msgs::RegionOfInterest2DArray roi_array_msg;
  // generate masked images for all objects and calculate contours
  for (int i = 0; i < masked_images_.size(); i++) {
    double min_h, max_h, min_s, max_s, min_v, max_v;
    threashold_params[i].get_threshold(min_h, max_h, min_s, max_s, min_v, max_v);
    // cv::inRange(hsv_image, cv::Scalar(min_h, min_s, min_v, 0) , cv::Scalar(max_h, max_s, max_v, 0),
    // masked_images_[i]);
    if (min_h < max_h) {
      cv::inRange(hsv_image, cv::Scalar(min_h, min_s, min_v, 0), cv::Scalar(max_h, max_s, max_v, 0),
                  masked_images_[i]);
    } else {
      cv::Mat lower_image;
      cv::inRange(hsv_image, cv::Scalar(min_h, min_s, min_v, 0), cv::Scalar(255, max_s, max_v, 0),
                  lower_image);
      cv::Mat upper_image;
      cv::inRange(hsv_image, cv::Scalar(0, min_s, min_v, 0), cv::Scalar(max_h, max_s, max_v, 0), upper_image);
      cv::bitwise_or(lower_image, upper_image, masked_images_[i]);
    }
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(masked_images_[i], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    double min_area, max_area;
    threashold_params[i].get_area_threshold(min_area, max_area);
    // check contour size for ALL contours
    for (std::vector<std::vector<cv::Point> >::const_iterator itr = contours.begin(); itr != contours.end();
         ++itr) {
      std::vector<cv::Point> approx;
      approxPolyDP(*itr, approx, 5, true);
      double area = contourArea(approx);
      if (max_area >= area && area >= min_area) {
        cv::Rect bbox_rect = cv::boundingRect(approx);
        robotx_msgs::RegionOfInterest2D roi_msg;
        // robotx_msgs::Object2D object2d_msg;
        // object2d_msg.type = threashold_params[i].target_buoy_name;
        // object2d_msg.boundingbox.header = msg->header;
        roi_msg.roi.x_offset = bbox_rect.x;
        roi_msg.roi.y_offset = bbox_rect.y;
        roi_msg.roi.width = bbox_rect.width;
        roi_msg.roi.height = bbox_rect.height;
        roi_array_msg.object_rois.push_back(roi_msg);
        // object2d_msg.boundingbox.corner_point_0[0] = bbox_rect.x;
        // object2d_msg.boundingbox.corner_point_0[1] = bbox_rect.y;
        // object2d_msg.boundingbox.corner_point_1[0] = bbox_rect.x + bbox_rect.width;
        // object2d_msg.boundingbox.corner_point_1[1] = bbox_rect.y + bbox_rect.height;
        // objects2d_msg.objects.push_back(object2d_msg);
      }
    }
  }
  roi_array_pub_.publish(roi_array_msg);
}
