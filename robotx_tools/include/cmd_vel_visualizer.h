#ifndef CMD_VEL_VISUALIZER_H_INCLUDED
#define CMD_VEL_VISUALIZER_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

class cmd_vel_visualizer
{
public:
    cmd_vel_visualizer();
    ~cmd_vel_visualizer();
private:
    ros::NodeHandle nh_;
    ros::Publisher linear_x_pub_;
    ros::Publisher linear_y_pub_;
    ros::Publisher angular_z_pub_;
    ros::Subscriber cmd_vel_sub_;
    void cmd_vel_callback_(const geometry_msgs::TwistConstPtr msg);
};

#endif  //CMD_VEL_VISUALIZER_H_INCLUDED