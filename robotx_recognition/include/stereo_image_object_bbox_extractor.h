#ifndef STEREO_IMAGE_OBJECT_BBOX_EXTRACTOR_H_INCLUDED
#define STEREO_IMAGE_OBJECT_BBOX_EXTRACTOR_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

//headers in this package
#include <disparity_image.h>

class stereo_image_object_bbox_extractor
{
public:
    struct parameters
    {
        std::string camera_base_link;
        std::string left_image_topic;
        std::string right_image_topic;
        std::string euclidean_cluster_topic;
        bool publish_disparity;
        double horizontal_fov;
        parameters()
        {
            publish_disparity = false;
        }
    };
    stereo_image_object_bbox_extractor();
    ~stereo_image_object_bbox_extractor();
    void set_parameters(parameters params){params_ = params;};
private:
    void left_image_callback_(const sensor_msgs::ImageConstPtr& msg);
    void right_image_callback_(const sensor_msgs::ImageConstPtr& msg);
    void euclidean_cluster_callback_(jsk_recognition_msgs::BoundingBoxArray msg);
    void setup_publisher_subscriber_();
    image_transport::Subscriber left_image_sub_,right_image_sub_;
    image_transport::Publisher disparity_image_pub_;
    ros::Subscriber euclidean_cluster_sub_;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    disparity_image disparity_image_;
    parameters params_;
    cv::Mat left_image_;
    cv::Mat right_image_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif  //STEREO_IMAGE_OBJECT_BBOX_EXTRACTOR_H_INCLUDED