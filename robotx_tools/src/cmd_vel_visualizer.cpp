#include <cmd_vel_visualizer.h>

cmd_vel_visualizer::cmd_vel_visualizer()
{
    linear_x_pub_ = nh_.advertise<std_msgs::Float32>("/cmd_vel/linear/x",1);
    linear_y_pub_ = nh_.advertise<std_msgs::Float32>("/cmd_vel/linear/y",1);
    angular_z_pub_ = nh_.advertise<std_msgs::Float32>("/cmd_vel/angular/z",1);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel",1,&cmd_vel_visualizer::cmd_vel_callback_,this);
}

cmd_vel_visualizer::~cmd_vel_visualizer()
{

}

void cmd_vel_visualizer::cmd_vel_callback_(const geometry_msgs::TwistConstPtr msg)
{
    std_msgs::Float32 pub_msg;
    pub_msg.data = msg->linear.x;
    linear_x_pub_.publish(pub_msg);
    pub_msg.data = msg->linear.y;
    linear_y_pub_.publish(pub_msg);
    pub_msg.data = msg->angular.z;
    angular_z_pub_.publish(pub_msg);
    return;
}