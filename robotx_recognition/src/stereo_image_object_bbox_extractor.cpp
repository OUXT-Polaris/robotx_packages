#include <stereo_image_object_bbox_extractor.h>
#include <disparity_image.h>

stereo_image_object_bbox_extractor::stereo_image_object_bbox_extractor() : it_(nh_), tf_listener_(tf_buffer_)
{
    disparity_image::parameters disparity_params;
    disparity_params.blockSize = 1;
    disparity_params.numDisparities = 16;
    disparity_params.P1 = 0;
    disparity_params.P2 = 0;
    disparity_params.speckleWindowSize = 100;
    disparity_params.speckleRange = 32;
    disparity_params.mode = cv::StereoSGBM::MODE_HH4;
    disparity_image_.set_parameters(disparity_params);

    nh_.getParam(ros::this_node::getName()+"/publish_disparity",params_.publish_disparity);
    nh_.param<double>(ros::this_node::getName()+"horizontal_fov", params_.horizontal_fov, 1.3962634);
    nh_.param<std::string>(ros::this_node::getName()+"/camera_base_link", params_.camera_base_link, ros::this_node::getName()+"/camera_base_link");
    nh_.param<std::string>(ros::this_node::getName()+"/left_image_topic", params_.left_image_topic, ros::this_node::getName()+"/left_image_raw");
    nh_.param<std::string>(ros::this_node::getName()+"/right_image_topic", params_.right_image_topic, ros::this_node::getName()+"/right_image_raw");
    nh_.param<std::string>(ros::this_node::getName()+"/euclidean_cluster_topic", params_.euclidean_cluster_topic, ros::this_node::getName()+"/euclidean_cluster_bbox");
    set_parameters(params_);
    setup_publisher_subscriber_();
}

stereo_image_object_bbox_extractor::~stereo_image_object_bbox_extractor()
{

}

void stereo_image_object_bbox_extractor::euclidean_cluster_callback_(jsk_recognition_msgs::BoundingBoxArray msg)
{
    cv::Mat disparity;
    if(disparity_image_.get_disparity_image(disparity) != false)
    {
        return;
    }
    double horizontal_fov = params_.horizontal_fov;
    double vertical_fov = params_.horizontal_fov/disparity.cols*disparity.rows;
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(params_.camera_base_link, msg.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }
    for(int i=0; i<msg.boxes.size(); i++)
    {
        geometry_msgs::PointStamped bbox_center_point;
        bbox_center_point.header = msg.header;
        bbox_center_point.point.x = msg.boxes[i].pose.position.x;
        bbox_center_point.point.y = msg.boxes[i].pose.position.y;
        bbox_center_point.point.z = msg.boxes[i].pose.position.z;
        tf2::doTransform(bbox_center_point, bbox_center_point, transform_stamped);
    }
}

void stereo_image_object_bbox_extractor::setup_publisher_subscriber_()
{
    if(params_.publish_disparity)
    {
        disparity_image_pub_ = it_.advertise(ros::this_node::getName()+"/disparity_image", 1);
    }
    left_image_sub_  = it_.subscribe(params_.left_image_topic, 1, &stereo_image_object_bbox_extractor::left_image_callback_, this);
    right_image_sub_ = it_.subscribe(params_.right_image_topic, 1, &stereo_image_object_bbox_extractor::right_image_callback_, this);
    euclidean_cluster_sub_ = nh_.subscribe(params_.euclidean_cluster_topic, 1, &stereo_image_object_bbox_extractor::euclidean_cluster_callback_, this);
}

void stereo_image_object_bbox_extractor::left_image_callback_(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
		left_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        disparity_image_.set_left_image(left_image_);
        cv::Mat disparity;
        if(params_.publish_disparity && disparity_image_.get_disparity_image(disparity))
        {
            double min, max;
            cv::minMaxLoc(disparity, &min, &max);
            disparity.convertTo(disparity, CV_8UC1, 255.0 / (max - min), -255.0 * min / (max - min));
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", disparity).toImageMsg();
            disparity_image_pub_.publish(msg);
        }
	}
	catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
}

void stereo_image_object_bbox_extractor::right_image_callback_(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
	    right_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        disparity_image_.set_right_image(right_image_);
	}
	catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
}