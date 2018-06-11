#ifndef ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE
#define ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>

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
         * @brief connection timeout [sec] with motor controller
         * 
         */
        int timeout;
        /**
         * @brief connection timeout [sec] with joystick controller
         * 
         */
        int controller_timeout;
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
            ros::param::param<int>(ros::this_node::getName()+"/controller_timeout", controller_timeout, 30);
            ros::param::param<int>(ros::this_node::getName()+"/frequency", frequency, 30);
        };
    };
    robotx_hardware_interface();
    ~robotx_hardware_interface();
private:
    /**
     * @brief ROS callback function for joystick
     * 
     * @param msg joystick command.
     */
    void joy_callback_(sensor_msgs::Joy msg);
    /**
     * @brief ROS callback function for motor command
     * 
     * @param msg [left_thruster_cmd left_thruster_joint_angle right_thruster_cmd right_thruster_joint_angle]
     */
    void motor_command_callback_(std_msgs::Float32MultiArray msg);
    ros::NodeHandle nh_;
    ros::Subscriber motor_command_sub_;
    ros::Publisher usv_drive_cmd_pub_;
    ros::Publisher left_thrust_joint_pub_;
    ros::Publisher right_thrust_joint_pub_;
    ros::Duration last_connection_left_motor_;
    ros::Duration last_connection_right_motor_;
    ros::Duration last_connection_controller_;
    const parameters params_;
    enum targets_{all=0,simulation=1,hardware=2};
    enum modes_{emergency=-1,manual=0,autopilot=1};
    enum controllers_{dualshock4=0};
};

#endif //ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE