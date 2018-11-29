#ifndef FIELD_MAP_CLICKER_H_INCLUDED
#define FIELD_MAP_CLICKER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

class field_map_clicker
{
public:
    field_map_clicker();
    ~field_map_clicker();
private:
    ros::NodeHandle nh_;
    ros::Subscriber green_buoy_sub_;
    void green_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg);
    std::vector<geometry_msgs::Point> green_buoy_positions_;
    ros::Subscriber red_buoy_sub_;
    void red_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg);
    std::vector<geometry_msgs::Point> red_buoy_positions_;
    ros::Subscriber white_buoy_sub_;
    void white_buoy_callback_(const geometry_msgs::PointStamped::ConstPtr msg);
    std::vector<geometry_msgs::Point> white_buoy_positions_;
};

#endif  //FIELD_MAP_CLICKER_H_INCLUDED