#ifndef WAYPOINT_CLICERK_H_INCLUDED
#define WAYPOINT_CLICERK_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

//headers in boost
#include <boost/thread.hpp>

//headers in STL
#include <fstream>

class waypoint_clicker
{
public:
    waypoint_clicker(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~waypoint_clicker();
    void run();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string waypoint_frame_;
    ros::Subscriber goal_pose_sub_;
    ros::Publisher marker_pub_;
    void publish_marker_();
    void goal_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    void add_goal_pose_(const geometry_msgs::PoseStamped::ConstPtr msg);
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<geometry_msgs::PoseStamped> target_poses_;
    std::vector<geometry_msgs::Pose2D> target_poses_2d_;
    bool initial_pose_received_;
    geometry_msgs::PoseStamped::ConstPtr initial_pose_;
};

#endif  //WAYPOINT_CLICERK_H_INCLUDED