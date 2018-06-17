#ifndef OBSTACLE_MAP_SERVER_H_INCLUDED
#define OBSTACLE_MAP_SERVER_H_INCLUDED

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <ros/ros.h>

/**
 * @brief obstacle_map_server class
 * 
 * this class recoeve /euclidian_clustering_node/marker topic and publish /obstacle_map topic
 */
class obstacle_map_server
{
    struct parameters
    {
        /**
         * @brief margin of objects [m]
         * 
         */
        double margin;
        /**
         * @brief map resolution [m/cell]
         * 
         */
        double resolution;
        /**
         * @brief map width [cells]
         * 
         */
        int map_width;
        /**
         * @brief map height [cells]
         * 
         */
        int map_height;
        parameters()
        {
            ros::param::param<double>(ros::this_node::getName()+"/margin", margin, 0.2);
            ros::param::param<double>(ros::this_node::getName()+"/resolution", resolution, 0.05);
            ros::param::param<int>(ros::this_node::getName()+"/map_height", map_height, 400);
            ros::param::param<int>(ros::this_node::getName()+"/map_width", map_width, 400);
        }
    };
public:
    obstacle_map_server();
    ~obstacle_map_server();
};

#endif  //OBSTACLE_MAP_SERVER_H_INCLUDED