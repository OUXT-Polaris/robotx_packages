#include <object_bbox_extractor.h>

object_bbox_extractor::object_bbox_extractor() : it_(nh_), params_(), tf_listener_(tf_buffer_)
{
    image_sub_ = it_.subscribe(params_.image_topic, 1, &object_bbox_extractor::image_callback_, this);
    euclidean_cluster_sub_ = nh_.subscribe(params_.euclidean_cluster_topic, 1, &object_bbox_extractor::euclidean_cluster_callback_, this);
}

object_bbox_extractor::~object_bbox_extractor()
{

}

void object_bbox_extractor::image_callback_(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(params_.camera_link, msg->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }
    for(int i=0; i<last_bbox_msg_.boxes.size(); i++)
    {
        cv::Rect rect;
        raycast_to_image(image,last_bbox_msg_.boxes[i],transform_stamped,rect);
    }
}

void object_bbox_extractor::euclidean_cluster_callback_(jsk_recognition_msgs::BoundingBoxArray msg)
{
    last_bbox_msg_ = msg;
}

bool object_bbox_extractor::raycast_to_image(cv::Mat image, jsk_recognition_msgs::BoundingBox object_bbox, 
    geometry_msgs::TransformStamped transform_stamped, cv::Rect& image_bbox)
{

}