#include <object_bbox_extractor.h>

object_bbox_extractor::object_bbox_extractor() : it_(nh_), params_(), tf_listener_(tf_buffer_) {
  roi_array_pub_ =
      nh_.advertise<robotx_msgs::ObjectRegionOfInterestArray>(ros::this_node::getName() + "/object_roi", 1);
  if (params_.publish_rect_image) {
    image_pub_ = it_.advertise(ros::this_node::getName() + "/rect_image", 1);
  }
  if (params_.enable_roi_image_publisher == true) {
    roi_image_pub_ = it_.advertise(ros::this_node::getName() + "/roi_image", 1);
  }
  image_sub_ = it_.subscribe(params_.image_topic, 1, &object_bbox_extractor::image_callback_, this);
  euclidean_cluster_sub_ = nh_.subscribe(params_.euclidean_cluster_topic, 1,
                                         &object_bbox_extractor::euclidean_cluster_callback_, this);
}

object_bbox_extractor::~object_bbox_extractor() {}

void object_bbox_extractor::image_callback_(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(params_.camera_link, msg->header.frame_id, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  cv::Mat rect_image = image.clone();
  robotx_msgs::ObjectRegionOfInterestArray roi_array_msg;
  for (int i = 0; i < last_bbox_msg_.boxes.size(); i++) {
    cv::Rect rect;
    if (raycast_to_image(image, last_bbox_msg_.boxes[i], transform_stamped, last_bbox_msg_.header, rect)) {
      robotx_msgs::ObjectRegionOfInterest roi_msg;
      roi_msg.header = msg->header;
      roi_msg.roi_2d.do_rectify = false;
      roi_msg.roi_2d.x_offset = rect.x;
      roi_msg.roi_2d.y_offset = rect.y;
      roi_msg.roi_2d.width = rect.width;
      roi_msg.roi_2d.height = rect.height;
      roi_msg.roi_3d = last_bbox_msg_.boxes[i];
      roi_array_msg.object_rois.push_back(roi_msg);
      if (params_.publish_rect_image) {
        cv::rectangle(rect_image, cv::Point(rect.x, rect.y),
                      cv::Point(rect.x + rect.width, rect.y + rect.height), cv::Scalar(0, 0, 200), 3, 4);
      }
      if (params_.enable_roi_image_publisher) {
        cv::Mat roi_image(image, rect);
        sensor_msgs::ImagePtr roi_image_msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi_image).toImageMsg();
        roi_image_pub_.publish(roi_image_msg);
      }
    }
  }
  if (params_.publish_rect_image) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rect_image).toImageMsg();
    image_pub_.publish(msg);
  }
  roi_array_pub_.publish(roi_array_msg);
}

void object_bbox_extractor::euclidean_cluster_callback_(jsk_recognition_msgs::BoundingBoxArray msg) {
  last_bbox_msg_ = msg;
}

bool object_bbox_extractor::raycast_to_image(cv::Mat image,
                                             jsk_recognition_msgs::BoundingBox object_bbox,
                                             geometry_msgs::TransformStamped transform_stamped,
                                             std_msgs::Header header,
                                             cv::Rect& image_bbox) {
  image_bbox = cv::Rect();
  double horizontal_fov = params_.horizontal_fov;
  double vertical_fov = params_.horizontal_fov / image.cols * image.rows;
  geometry_msgs::PointStamped bbox_center_point;
  bbox_center_point.header = header;
  bbox_center_point.point.x = object_bbox.pose.position.x;
  bbox_center_point.point.y = object_bbox.pose.position.y;
  bbox_center_point.point.z = object_bbox.pose.position.z;
  tf2::doTransform(bbox_center_point, bbox_center_point, transform_stamped);
  std::array<geometry_msgs::PointStamped, 4> bbox_points;
  std::array<cv::Point, 4> image_rect_points;
  for (int i = 0; i < 4; i++) {
    if (i == 0) {
      bbox_points[i].header = header;
      bbox_points[i].point.x = object_bbox.pose.position.x + object_bbox.dimensions.x / 2;
      bbox_points[i].point.y = object_bbox.pose.position.y + object_bbox.dimensions.y;
      bbox_points[i].point.z = object_bbox.pose.position.z + object_bbox.dimensions.z;
    }
    if (i == 1) {
      bbox_points[i].header = header;
      bbox_points[i].point.x = object_bbox.pose.position.x + object_bbox.dimensions.x / 2;
      bbox_points[i].point.y = object_bbox.pose.position.y - object_bbox.dimensions.y;
      bbox_points[i].point.z = object_bbox.pose.position.z + object_bbox.dimensions.z;
    }
    if (i == 2) {
      bbox_points[i].header = header;
      bbox_points[i].point.x = object_bbox.pose.position.x + object_bbox.dimensions.x / 2;
      bbox_points[i].point.y = object_bbox.pose.position.y + object_bbox.dimensions.y;
      bbox_points[i].point.z = object_bbox.pose.position.z - object_bbox.dimensions.z;
    }
    if (i == 3) {
      bbox_points[i].header = header;
      bbox_points[i].point.x = object_bbox.pose.position.x + object_bbox.dimensions.x / 2;
      bbox_points[i].point.y = object_bbox.pose.position.y - object_bbox.dimensions.y;
      bbox_points[i].point.z = object_bbox.pose.position.z - object_bbox.dimensions.z;
    }
    tf2::doTransform(bbox_points[i], bbox_points[i], transform_stamped);
    double pitch = std::atan2(bbox_points[i].point.z, bbox_points[i].point.x);
    double yaw = std::atan2(bbox_points[i].point.y, bbox_points[i].point.x);
    cv::Point image_point;
    image_point.x = image.cols / 2 - yaw / horizontal_fov * image.cols;
    image_point.y = image.rows / 2 - pitch / vertical_fov * image.rows;
    image_rect_points[i] = image_point;
  }
  if (image_rect_points[0].y < 0 && image_rect_points[1].y < 0)
    if (image_rect_points[1].y < 0 && image_rect_points[2].y < 0) return false;
  if (image_rect_points[0].y > (image.rows - 1) && image_rect_points[1].y > (image.rows - 1))
    if (image_rect_points[1].y > (image.rows - 1) && image_rect_points[2].y > (image.rows - 1)) return false;
  if (image_rect_points[0].x < 0 && image_rect_points[1].x < 0)
    if (image_rect_points[1].x < 0 && image_rect_points[2].x < 0) return false;
  if (image_rect_points[0].x > (image.cols - 1) && image_rect_points[1].x > (image.cols - 1))
    if (image_rect_points[1].x > (image.cols - 1) && image_rect_points[2].x > (image.cols - 1)) return false;
  for (int i = 0; i < 4; i++) {
    if (image_rect_points[i].x < 0) image_rect_points[i].x = 0;
    if (image_rect_points[i].x > (image.cols - 1)) image_rect_points[i].x = image.cols - 1;
    if (image_rect_points[i].y < 0) image_rect_points[i].y = 0;
    if (image_rect_points[i].y > (image.rows - 1)) image_rect_points[i].y = image.rows - 1;
  }
  image_bbox.x = std::min(std::min(image_rect_points[0].x, image_rect_points[1].x),
                          std::min(image_rect_points[2].x, image_rect_points[3].x));
  image_bbox.y = std::min(std::min(image_rect_points[0].y, image_rect_points[1].y),
                          std::min(image_rect_points[2].y, image_rect_points[3].y));
  image_bbox.width = std::max(std::max(image_rect_points[0].x, image_rect_points[1].x),
                              std::max(image_rect_points[2].x, image_rect_points[3].x)) -
                     image_bbox.x;
  image_bbox.height = std::max(std::max(image_rect_points[0].y, image_rect_points[1].y),
                               std::max(image_rect_points[2].y, image_rect_points[3].y)) -
                      image_bbox.y;
  return true;
}