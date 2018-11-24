#include <navi_sim.h>

navi_sim::navi_sim()
{
    nh_ = ros::NodeHandle();
    pnh_ = ros::NodeHandle("~");
    pnh_.param<double>("update_rate", update_rate_, 10);
    pnh_.param<std::string>("world_frame", world_frame_, "world");
}

navi_sim::~navi_sim()
{
    
}

void navi_sim::twist_callback_(const geometry_msgs::Twist::ConstPtr msg)
{
    twist_cmd_ = *msg;
    return;
}

void navi_sim::update_()
{
    ros::Rate rate(update_rate_);
    while(ros::ok())
    {
        rate.sleep();
    }
    return;
}

void navi_sim::run()
{
    return;
}