#include <obstacle_map_server.h>

obstacle_map_server::obstacle_map_server() : params_()
{
    objects_bbox_sub_ = nh_.subscribe(params_.object_bbox_topic, 1, &obstacle_map_server::objects_bbox_callback_, this);
}

obstacle_map_server::~obstacle_map_server()
{

}

void obstacle_map_server::objects_bbox_callback_(jsk_recognition_msgs::BoundingBoxArray msg)
{

}