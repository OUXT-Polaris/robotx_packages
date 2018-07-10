#ifndef OBJECT_BBOX_EXTRACTOR_H_INCLUDED
#define OBJECT_BBOX_EXTRACTOR_H_INCLUDED

// headers in this package
#include <robotx_msgs/ObjectRegionOfInterestArray.h>

// headers in ROS
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class object_bbox_extractor {
 public:
  struct parameters {
    std::string camera_link;
    std::string image_topic;
    std::string euclidean_cluster_topic;
    double horizontal_fov;
    bool publish_rect_image;
    bool enable_roi_image_publisher;
    parameters() {
      ros::param::param<std::string>(ros::this_node::getName() + "/camera_link", camera_link, "camera_link");
      ros::param::param<std::string>(ros::this_node::getName() + "/image_topic", image_topic,
                                     ros::this_node::getName() + "/image_raw");
      ros::param::param<std::string>(ros::this_node::getName() + "/euclidean_cluster_topic",
                                     euclidean_cluster_topic,
                                     ros::this_node::getName() + "/euclidean_cluster/bbox");
      ros::param::param<double>(ros::this_node::getName() + "/horizontal_fov", horizontal_fov, 1.3962634);
      ros::param::param<bool>(ros::this_node::getName() + "/publish_rect_image", publish_rect_image, false);
      ros::param::param<bool>(ros::this_node::getName() + "/enable_roi_image_publisher",
                              enable_roi_image_publisher, false);
    }
  };
  object_bbox_extractor();
  ~object_bbox_extractor();

 private:
  void image_callback_(const sensor_msgs::ImageConstPtr& msg);
  void euclidean_cluster_callback_(jsk_recognition_msgs::BoundingBoxArray msg);
  bool raycast_to_image(cv::Mat image,
                        jsk_recognition_msgs::BoundingBox object_bbox,
                        geometry_msgs::TransformStamped transform_stamped,
                        std_msgs::Header header,
                        cv::Rect& image_bbox);
  const parameters params_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher roi_image_pub_;
  ros::Subscriber euclidean_cluster_sub_;
  ros::Publisher roi_array_pub_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  jsk_recognition_msgs::BoundingBoxArray last_bbox_msg_;
};
#endif  // OBJECT_BBOX_EXTRACTOR_H_INCLUDED