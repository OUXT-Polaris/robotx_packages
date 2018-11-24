#ifndef NAVI_SIM_H_INCLUDED
#define NAVI_SIM_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

//headers in this package
#include <UTM.h>

class navi_sim
{
public:
    navi_sim();
    ~navi_sim();
    void run();
private:
    geometry_msgs::Twist twist_cmd_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    geometry_msgs::Pose2D current_pose_;
    std::string world_frame_;
    double update_rate_;
    void update_();
    void twist_callback_(const geometry_msgs::Twist::ConstPtr msg);
};
#endif  //NAVI_SIM_H_INCLUDED