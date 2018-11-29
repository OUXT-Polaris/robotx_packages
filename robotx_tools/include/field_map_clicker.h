#ifndef FIELD_MAP_CLICKER_H_INCLUDED
#define FIELD_MAP_CLICKER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//headers in robotx_packages
#include <robotx_msgs/FieldMap.h>

//headers in STL
#include <fstream>

class field_map_clicker
{
public:
    field_map_clicker();
    ~field_map_clicker();
private:
    std::string map_frame_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber green_buoy_sub_;
    void green_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg);
    std::vector<geometry_msgs::Point> green_buoy_positions_;
    ros::Subscriber red_buoy_sub_;
    void red_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg);
    std::vector<geometry_msgs::Point> red_buoy_positions_;
    ros::Subscriber white_buoy_sub_;
    void white_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg);
    std::vector<geometry_msgs::Point> white_buoy_positions_;
    void save_();
    robotx_msgs::FieldMap field_map_;
};

#endif  //FIELD_MAP_CLICKER_H_INCLUDED