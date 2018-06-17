#include <obstacle_map_server.h>
#include <tf/transform_datatypes.h>

obstacle_map_server::obstacle_map_server() : params_()
{
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/obstacle_map", 1);
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
    map.info.origin.position.x = -params_.map_height*params_.resolution/2;
    map.info.origin.position.y = -params_.map_width*params_.resolution/2;
    map.info.origin.position.z = params_.height_offset;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    map.info.origin.orientation.w = 1;
    std::vector<int8_t> map_data(params_.map_height*params_.map_width);
    std::vector<std::array<double,3> > objects_data;
    for(int i = 0; i<msg.boxes.size() ;i++)
    {
        std::array<double,3> object_data;
        double radius = std::max(msg.boxes[i].dimensions.x, msg.boxes[i].dimensions.y) + params_.margin;
        double center_x = msg.boxes[i].pose.position.x;
        double center_y = msg.boxes[i].pose.position.y;
        object_data[0] = radius;
        object_data[1] = center_x;
        object_data[2] = center_y;
        objects_data.push_back(object_data);
    }
    for(int i=0; i<params_.map_height; i++)
    {
        for(int m=0; m<params_.map_width; m++)
        {
            double x = (double)m * params_.resolution - 0.5 * params_.resolution * params_.map_width;
            double y = (double)i * params_.resolution - 0.5 * params_.resolution * params_.map_height;
            for(int n=0; n<objects_data.size(); n++)
            {
                double distance = std::sqrt(std::pow(objects_data[n][1]-x,2) + std::pow(objects_data[n][2]-y,2));
                if(distance < objects_data[n][0])
                    map_data[i*params_.map_height+m] = 100;
            }
        }
    }
    map.data = map_data;
    map_pub_.publish(map);
    return map;
}