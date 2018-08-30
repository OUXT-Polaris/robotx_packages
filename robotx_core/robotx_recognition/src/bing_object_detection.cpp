#include <bing_object_detection.h>

#include <ros/package.h>

#include <algorithm>

bing_object_detection::bing_object_detection() : _it(_nh) {
  _bbox_pub = _nh.advertise<robotx_msgs::RegionOfInterest2DArray>(ros::this_node::getName() + "/bbox", 1);
  ros::param::param<int>(ros::this_node::getName() + "/max_num_bbox", _max_num_bbox, 10);
  _image_sub = _it.subscribe(ros::this_node::getName() + "/image_raw", 1,
                             &bing_object_detection::_image_callback, this);
}

bing_object_detection::~bing_object_detection() {}

void bing_object_detection::_detect(cv::Mat image,
                                    std::vector<cv::Vec4i>& bboxs,
                                    std::vector<float>& objectness_vals) {
  setTrainingPath(ros::package::getPath("robotx_recognition") + "/data/bing");
  computeSaliencyImpl(image, bboxs);
  objectness_vals = getobjectnessValues();
}

void bing_object_detection::_image_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat image;
  try {
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::vector<cv::Vec4i> bboxs;
  std::vector<float> objectness_vals;
  _detect(image, bboxs, objectness_vals);
  // robotx_msgs::BoundingBox2DArrayStamped bbox_msg;
  robotx_msgs::RegionOfInterest2DArray bbox_msg;
  std::vector<robotx_msgs::RegionOfInterest2D> all_bbox;
  // bbox_msg.header = msg->header;
  for (int i = 0; i < bboxs.size(); i++) {
    /*
    robotx_msgs::BoundingBox2D single_bbox;
    single_bbox.objectness = objectness_vals[i];
    single_bbox.corner_point_0[0] = bboxs[i][0];
    single_bbox.corner_point_0[1] = bboxs[i][1];
    single_bbox.corner_point_1[0] = bboxs[i][2];
    single_bbox.corner_point_1[1] = bboxs[i][3];
    */
    // sensor_msgs::RegionOfInterest single_bbox;
    // all_bbox.push_back(single_bbox);
    robotx_msgs::RegionOfInterest2D single_bbox;
    single_bbox.objectness = objectness_vals[i];
    single_bbox.roi.x_offset = bboxs[i][0];
    single_bbox.roi.y_offset = bboxs[i][1];
    single_bbox.roi.width = bboxs[i][2] - bboxs[i][0];
    single_bbox.roi.height = bboxs[i][3] - bboxs[i][1];
    all_bbox.push_back(single_bbox);
  }
  sort(all_bbox.begin(), all_bbox.end(),
       [](const robotx_msgs::RegionOfInterest2D& x, const robotx_msgs::RegionOfInterest2D& y) {
         return x.objectness > y.objectness;
       });
  if (all_bbox.size() <= _max_num_bbox) {
    // bbox_msg.bounding_boxes = all_bbox;
    bbox_msg.object_rois = all_bbox;
  } else {
    for (int i = 0; i < _max_num_bbox; i++) {
      bbox_msg.object_rois.push_back(all_bbox[i]);
      // bbox_msg.bounding_boxes.push_back(all_bbox[i]);
    }
  }
  _bbox_pub.publish(bbox_msg);

  // visualize bbox
  /*
  for(int i= 0; i<bbox_msg.bounding_boxes.size(); i++)
  {
      cv::rectangle(image,
          cv::Point(bbox_msg.bounding_boxes[i].corner_point_0[0],bbox_msg.bounding_boxes[i].corner_point_0[1]),
          cv::Point(bbox_msg.bounding_boxes[i].corner_point_1[0],bbox_msg.bounding_boxes[i].corner_point_1[1]),
  cv::Scalar(0,0,200), 1, 4);
  }
  cv::imshow("test",image);
  cv::waitKey(3);
  */
}