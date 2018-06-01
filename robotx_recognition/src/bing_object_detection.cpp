#include <bing_object_detection.h>
#include <ros/package.h>

bing_object_detection::bing_object_detection() : _it(_nh)
{
    _bbox_pub = _nh.advertise<robotx_msgs::BoundingBox2DArrayStamped>(ros::this_node::getName()+"/bbox", 1);
    _image_sub = _it.subscribe(ros::this_node::getName()+"/image_raw", 1, &bing_object_detection::_image_callback, this);
}

bing_object_detection::~bing_object_detection()
{

}

void bing_object_detection::_detect(cv::Mat image, std::vector<cv::Vec4i>& bboxs, std::vector<float>& objectness_vals)
{
    setTrainingPath(ros::package::getPath("robotx_recognition")+"/data/bing");
    computeSaliencyImpl(image, bboxs);
    objectness_vals = getobjectnessValues();
}

void bing_object_detection::_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat image;
	try
    {
		image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	}
	catch (cv_bridge::Exception& e)
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
	}
    std::vector<cv::Vec4i> bboxs;
    std::vector<float> objectness_vals;
    _detect(image,bboxs,objectness_vals);
    robotx_msgs::BoundingBox2DArrayStamped bbox_msg;
    bbox_msg.header = msg->header;
    for(int i=0; i<bboxs.size(); i++)
    {
        robotx_msgs::BoundingBox2D single_bbox;
        single_bbox.objectness = objectness_vals[i];
        single_bbox.corner_point_0[0] = bboxs[i][0];
        single_bbox.corner_point_0[1] = bboxs[i][1];
        single_bbox.corner_point_1[0] = bboxs[i][2];
        single_bbox.corner_point_1[1] = bboxs[i][3];
        bbox_msg.bounding_boxes.push_back(single_bbox);
    }
    _bbox_pub.publish(bbox_msg);
}