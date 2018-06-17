#ifndef OBSTACLE_MAP_SERVER_H_INCLUDED
#define OBSTACLE_MAP_SERVER_H_INCLUDED

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

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
         * @brief min object position in Z axis [m]
         * 
         */
        double min_object_position_z;
        /**
         * @brief max object position in Z axis [m]
         * 
         */
        double max_object_position_z;
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
        /**
         * @brief map origin in X axis
         * 
         */
        double origin_x;
        /**
         * @brief map origin in Y axis
         * 
         */
        double origin_y;
        /**
         * @brief map origin in theta
         * 
         */
        double origin_theta;
        /**
         * @brief ROS topic name for object bounding box
         * 
         */
        std::string object_bbox_topic;
        /**
         * @brief Construct a new parameters object
         * 
         */
        parameters()
        {
            ros::param::param<double>(ros::this_node::getName()+"/margin", margin, 0.2);
            ros::param::param<double>(ros::this_node::getName()+"/resolution", resolution, 0.05);
            ros::param::param<int>(ros::this_node::getName()+"/map_height", map_height, 400);
            ros::param::param<int>(ros::this_node::getName()+"/map_width", map_width, 400);
            ros::param::param<double>(ros::this_node::getName()+"/origin_x", origin_x, 0.0);
            ros::param::param<double>(ros::this_node::getName()+"/origin_y", origin_y, 0.0);
            ros::param::param<double>(ros::this_node::getName()+"/origin_theta", origin_theta, 0.0);
            ros::param::param<double>(ros::this_node::getName()+"/min_object_position_z", min_object_position_z, -3.0);
            ros::param::param<double>(ros::this_node::getName()+"/max_object_position_z", max_object_position_z, 3.0);
            ros::param::param<std::string>(ros::this_node::getName()+"/object_bbox_topic", object_bbox_topic, ros::this_node::getName()+"/object_bbox");
        }
    };
public:
    /**
     * @brief Construct a new obstacle map server object
     * 
     */
    obstacle_map_server();
    /**
     * @brief Destroy the obstacle map server object
     * 
     */
    ~obstacle_map_server();
private:
    /**
     * @brief parameters for obstacle_map_server class
     * 
     */
    const parameters params_;
    /**
     * @brief ROS NodeHandle
     * 
     */
    ros::NodeHandle nh_;
    /**
     * @brief ROS subscriber for (object_bbox_topic) topic (message type : jsk_recognition_msgs/BoundingBoxArray)
     * 
     */
    ros::Subscriber objects_bbox_sub_;
    /**
     * @brief ROS callback function for (object_bbox_topic) topic (message type : jsk_recognition_msgs/BoundingBoxArray)
     * 
     * @param msg ROS message (message type : jsk_recognition_msgs/BoundingBoxArray)
     * @sa obstacle_map_server::objects_bbox_sub_
     */
    void objects_bbox_callback_(jsk_recognition_msgs::BoundingBoxArray msg);
};

#endif  //OBSTACLE_MAP_SERVER_H_INCLUDED