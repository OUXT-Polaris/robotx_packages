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
        raycast_to_image(image,last_bbox_msg_.boxes[i],transform_stamped,last_bbox_msg_.header,rect);
    }
}

void object_bbox_extractor::euclidean_cluster_callback_(jsk_recognition_msgs::BoundingBoxArray msg)
{
    last_bbox_msg_ = msg;
}

bool object_bbox_extractor::raycast_to_image(cv::Mat image, jsk_recognition_msgs::BoundingBox object_bbox, 
    geometry_msgs::TransformStamped transform_stamped, std_msgs::Header header, cv::Rect& image_bbox)
{
    double horizontal_fov = params_.horizontal_fov;
    double vertical_fov = params_.horizontal_fov/image.cols*image.rows;
    std::array<geometry_msgs::PointStamped,8> bbox_points;
    for(int i=0; i<8; i++)
    {
        if(i = 0)
        {
            bbox_points[i].header = header;
            bbox_points[i].point.x = object_bbox.pose.position.x + object_bbox.dimensions.x;
            bbox_points[i].point.y = object_bbox.pose.position.y + object_bbox.dimensions.y;
            bbox_points[i].point.z = object_bbox.pose.position.z + object_bbox.dimensions.z;
        }
        if(i == 1)
        {
            bbox_points[i].header = header;
            bbox_points[i].point.x = object_bbox.pose.position.x - object_bbox.dimensions.x;
            bbox_points[i].point.y = object_bbox.pose.position.y + object_bbox.dimensions.y;
            bbox_points[i].point.z = object_bbox.pose.position.z + object_bbox.dimensions.z;
        }
        if(i == 2)
        {
            bbox_points[i].header = header;
            bbox_points[i].point.x = object_bbox.pose.position.x + object_bbox.dimensions.x;
            bbox_points[i].point.y = object_bbox.pose.position.y - object_bbox.dimensions.y;
            bbox_points[i].point.z = object_bbox.pose.position.z + object_bbox.dimensions.z;
        }
        if(i == 3)
        {
            bbox_points[i].header = header;
            bbox_points[i].point.x = object_bbox.pose.position.x + object_bbox.dimensions.x;
            bbox_points[i].point.y = object_bbox.pose.position.y + object_bbox.dimensions.y;
            bbox_points[i].point.z = object_bbox.pose.position.z - object_bbox.dimensions.z;
        }
        if(i == 4)
        {
            bbox_points[i].header = header;
            bbox_points[i].point.x = object_bbox.pose.position.x - object_bbox.dimensions.x;
            bbox_points[i].point.y = object_bbox.pose.position.y - object_bbox.dimensions.y;
            bbox_points[i].point.z = object_bbox.pose.position.z + object_bbox.dimensions.z;
        }
        if(i == 5)
        {
            bbox_points[i].header = header;
            bbox_points[i].point.x = object_bbox.pose.position.x - object_bbox.dimensions.x;
            bbox_points[i].point.y = object_bbox.pose.position.y + object_bbox.dimensions.y;
            bbox_points[i].point.z = object_bbox.pose.position.z - object_bbox.dimensions.z;
        }
        if(i == 6)
        {
            bbox_points[i].header = header;
            bbox_points[i].point.x = object_bbox.pose.position.x + object_bbox.dimensions.x;
            bbox_points[i].point.y = object_bbox.pose.position.y - object_bbox.dimensions.y;
            bbox_points[i].point.z = object_bbox.pose.position.z - object_bbox.dimensions.z;
        }
        if(i == 7)
        {
            bbox_points[i].header = header;
            bbox_points[i].point.x = object_bbox.pose.position.x - object_bbox.dimensions.x;
            bbox_points[i].point.y = object_bbox.pose.position.y - object_bbox.dimensions.y;
            bbox_points[i].point.z = object_bbox.pose.position.z - object_bbox.dimensions.z;
        }
        tf2::doTransform(bbox_points[i], bbox_points[i], transform_stamped);
    }
}