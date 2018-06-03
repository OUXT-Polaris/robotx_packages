//headers for ros
#include <ros/ros.h>

//headers in this package
#include <stereo_image_object_bbox_extractor.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stereo_image_object_bbox_extractor_node");
    stereo_image_object_bbox_extractor extractor;
    ros::spin();
    return 0;
}