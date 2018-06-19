#ifndef ROBOTX_LOCALIZATION_H_INCLUDED
#define ROBOTX_LOCALIZATION_H_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>

class robotx_localization
{
public:
    struct parameters
    {
        std::string publish_frame;
        std::string twist_topic;
        std::string fix_topic;
        double system_noise_x;
        double system_noise_y;
        double system_noise_theta;
        double measurement_noise_x;
        double measurement_noise_y;
        parameters()
        {
            ros::param::param<std::string>(ros::this_node::getName()+"/publish_frame", publish_frame, "map");
            ros::param::param<std::string>(ros::this_node::getName()+"/twist_topic", twist_topic, ros::this_node::getName()+"twist");
            ros::param::param<std::string>(ros::this_node::getName()+"/fix_topic", fix_topic, ros::this_node::getName()+"fix");
            ros::param::param<double>(ros::this_node::getName()+"/system_noise/x", system_noise_x, 0.05);
            ros::param::param<double>(ros::this_node::getName()+"/system_noise/y", system_noise_y, 0.05);
            ros::param::param<double>(ros::this_node::getName()+"/system_noise/theta", system_noise_theta, 0.05);
            ros::param::param<double>(ros::this_node::getName()+"/measurement_noise/x", measurement_noise_x, 0.5);
            ros::param::param<double>(ros::this_node::getName()+"/measurement_noise/y", measurement_noise_y, 0.5);
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
    Eigen::MatrixXd transition_matrix_;
    Eigen::MatrixXd input_matrix_;
    Eigen::MatrixXd measurement_matrix_;
    // noise matrix
    Eigen::MatrixXd system_noise_matrix_;
    Eigen::MatrixXd measurement_noise_matrix_;
    sensor_msgs::NavSatFix init_measurement_;
    volatile bool fix_recieved;
    volatile bool twist_received;
};
#endif  //ROBOTX_LOCALIZATION_H_INCLUDED