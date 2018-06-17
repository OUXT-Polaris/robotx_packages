#include <obstacle_map_server.h>
#include <tf/transform_datatypes.h>

obstacle_map_server::obstacle_map_server() : params_()
{
    objects_bbox_sub_ = nh_.subscribe(params_.object_bbox_topic, 1, &obstacle_map_server::objects_bbox_callback_, this);
}

obstacle_map_server::~obstacle_map_server()
{

}

void obstacle_map_server::objects_bbox_callback_(jsk_recognition_msgs::BoundingBoxArray msg)
{
    nav_msgs::OccupancyGrid map = generate_occupancy_grid_map(msg);
}

nav_msgs::OccupancyGrid obstacle_map_server::generate_occupancy_grid_map(jsk_recognition_msgs::BoundingBoxArray msg)
{
    nav_msgs::OccupancyGrid map;
    map.header = msg.header;
    map.info.map_load_time = ros::Time::now();
    map.info.height = params_.map_height;
    map.info.width = params_.map_width;
    map.info.resolution = params_.resolution;
    map.info.origin.position.x = params_.origin_x;
    map.info.origin.position.y = params_.origin_y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,params_.origin_theta);
    map.info.origin.orientation.x = quaternion.getX();
    map.info.origin.orientation.y = quaternion.getY();
    map.info.origin.orientation.z = quaternion.getZ();
    map.info.origin.orientation.w = quaternion.getW();
    return map;
}