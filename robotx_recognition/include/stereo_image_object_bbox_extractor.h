#ifndef STEREO_IMAGE_OBJECT_BBOX_EXTRACTOR_H_INCLUDED
#define STEREO_IMAGE_OBJECT_BBOX_EXTRACTOR_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//headers in this package
#include <disparity_image.h>

class stereo_image_object_bbox_extractor
{
public:
    struct config
    {
        std::string left_image_topic;
        std::string right_image_topic;
        bool publish_disparity;
        config()
        {
            publish_disparity = false;
        }
    };
    stereo_image_object_bbox_extractor();
    ~stereo_image_object_bbox_extractor();
    void left_image_callback(const sensor_msgs::ImageConstPtr& msg);
    void right_image_callback(const sensor_msgs::ImageConstPtr& msg);
    void set_config(config conf){config_ = conf;};
private:
    void setup_publisher_subscriber();
    image_transport::Subscriber left_image_sub_,right_image_sub_;
    image_transport::Publisher disparity_image_pub_;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    disparity_image disparity_image_;
    config config_;
    cv::Mat left_image_;
    cv::Mat right_image_;
};

#endif  //STEREO_IMAGE_OBJECT_BBOX_EXTRACTOR_H_INCLUDED