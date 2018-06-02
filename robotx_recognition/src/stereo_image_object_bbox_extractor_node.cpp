//headers for ros
#include <ros/ros.h>

//headers in this package
#include <disparity_image.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stereo_image_object_bbox_extractor_node");
    return 0;
}