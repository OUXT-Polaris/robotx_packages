#include <stereo_image_object_bbox_extractor.h>
#include <disparity_image.h>

stereo_image_object_bbox_extractor::stereo_image_object_bbox_extractor() : it_(nh_)
{
    disparity_image::parameters params;
    params.blockSize = 1;
    params.numDisparities = 16;
    params.P1 = 0;
    params.P2 = 0;
    params.speckleWindowSize = 100;
    params.speckleRange = 32;
    params.mode = cv::StereoSGBM::MODE_HH4;
    disparity_image_.set_parameters(params);

    config conf;
    nh_.getParam(ros::this_node::getName()+"/publish_disparity",conf.publish_disparity);
    nh_.param<std::string>(ros::this_node::getName()+"/left_image_topic",  conf.left_image_topic,  ros::this_node::getName()+"/left_image_raw");
    nh_.param<std::string>(ros::this_node::getName()+"/right_image_topic", conf.right_image_topic, ros::this_node::getName()+"/right_image_raw");
    set_config(conf);
    setup_publisher_subscriber();
}

stereo_image_object_bbox_extractor::~stereo_image_object_bbox_extractor()
{

}

void stereo_image_object_bbox_extractor::setup_publisher_subscriber()
{
    if(config_.publish_disparity)
    {
        disparity_image_pub_ = it_.advertise(ros::this_node::getName()+"/disparity_image", 1);
    }
    left_image_sub_  = it_.subscribe(config_.left_image_topic,  1, &stereo_image_object_bbox_extractor::left_image_callback,  this);
    right_image_sub_ = it_.subscribe(config_.right_image_topic, 1, &stereo_image_object_bbox_extractor::right_image_callback, this);
}

void stereo_image_object_bbox_extractor::left_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
		cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        disparity_image_.set_left_image(image);
        cv::Mat disparity;
        if(config_.publish_disparity && disparity_image_.get_disparity_image(disparity))
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

void stereo_image_object_bbox_extractor::right_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
		cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        disparity_image_.set_right_image(image);
	}
	catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
}