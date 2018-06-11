#ifndef ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE
#define ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE

#include <ros/ros.h>

class robotx_hardware_interface
{
public:
    struct parameters
    {
        /**
         * @brief hardware target
         * @sa robotx_hardware_interface::targets_
         */
        int target;
        /**
         * @brief action mode
         * @sa robotx_hardware_interface::modes_
         */
        int mode;
        /**
         * @brief connection timeout [sec]
         * 
         */
        int timeout;
        /**
         * @brief controller type
         * @sa robotx_hardware_interface::controllers_
         */
        int controller;
        /**
         * @brief frequency for publishing TCP/IP command.
         * 
         */
        int frequency;
        /**
         * @brief set parameters
         * 
         */
        parameters()
        {
            ros::param::param<int>(ros::this_node::getName()+"/mode", target, all);
            ros::param::param<int>(ros::this_node::getName()+"/mode", mode, manual);
            ros::param::param<int>(ros::this_node::getName()+"/timeout", timeout, 30);
            ros::param::param<int>(ros::this_node::getName()+"/controller", controller, dualshock4);
            ros::param::param<int>(ros::this_node::getName()+"/frequency", frequency, 30);
        };
    };
    robotx_hardware_interface();
    ~robotx_hardware_interface();
private:
    ros::NodeHandle nh_;
    ros::Subscriber left_thrust_power_sub_;
    ros::Subscriber left_thrust_joint_sub_;
    ros::Subscriber right_thrust_power_sub_;
    ros::Subscriber right_thrust_joint_sub_;
    ros::Publisher usv_drive_cmd_pub_;
    ros::Publisher left_thrust_joint_pub_;
    ros::Publisher right_thrust_joint_pub_;
    const parameters params_;
    enum targets_{all=0,simulation=1,hardware=2};
    enum modes_{emergency=-1,manual=0,autopilot=1};
    enum controllers_{dualshock4=0};
};

#endif //ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE