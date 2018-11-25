#ifndef WAYPOINT_CLICERK_H_INCLUDED
#define WAYPOINT_CLICERK_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

class waypoint_clicker
{
public:
    waypoint_clicker(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~waypoint_clicker();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string world_frame_;
    ros::Subscriber goal_pose_sub_;
    void goal_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif  //WAYPOINT_CLICERK_H_INCLUDED