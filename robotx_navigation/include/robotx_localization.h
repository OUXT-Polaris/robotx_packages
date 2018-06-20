#ifndef ROBOTX_LOCALIZATION_H_INCLUDED
#define ROBOTX_LOCALIZATION_H_INCLUDED

//headers in this package
#include <particle_filter.h>

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>

class robotx_localization
{
public:
    struct parameters
    {
        std::string publish_frame;
        std::string twist_topic;
        std::string fix_topic;
        int num_particles;
        int publish_rate;
        double min_x;
        double min_y;
        double max_x;
        double max_y;
        parameters()
        {
            ros::param::param<std::string>(ros::this_node::getName()+"/publish_frame", publish_frame, "map");
            ros::param::param<std::string>(ros::this_node::getName()+"/twist_topic", twist_topic, ros::this_node::getName()+"/twist");
            ros::param::param<std::string>(ros::this_node::getName()+"/fix_topic", fix_topic, ros::this_node::getName()+"/fix");
            ros::param::param<int>(ros::this_node::getName()+"/num_particles", num_particles, 1000);
            ros::param::param<int>(ros::this_node::getName()+"/publish_rate", publish_rate, 100);
            ros::param::param<double>(ros::this_node::getName()+"/min_x", min_x, -100);
            ros::param::param<double>(ros::this_node::getName()+"/min_y", min_y, -100);
            ros::param::param<double>(ros::this_node::getName()+"/max_x", max_x, 100);
            ros::param::param<double>(ros::this_node::getName()+"/max_y", max_y, 100);
        }
    };
    robotx_localization();
    ~robotx_localization();
private:
    const parameters params_;
    void fix_callback_(sensor_msgs::NavSatFix msg);
    void twist_callback_(geometry_msgs::Twist msg);
    ros::Subscriber fix_sub_;
    ros::Subscriber twist_sub_;
    ros::NodeHandle nh_;
    sensor_msgs::NavSatFix last_fix_message_;
    sensor_msgs::NavSatFix init_measurement_;
    geometry_msgs::Twist last_twist_message_;
    volatile bool fix_recieved_;
    volatile bool twist_received_;
    particle_filter* pfilter_ptr_;
    
};
#endif  //ROBOTX_LOCALIZATION_H_INCLUDED