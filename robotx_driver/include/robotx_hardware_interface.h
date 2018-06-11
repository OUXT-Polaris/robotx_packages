#ifndef ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE
#define ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE

//headers in this package
#include <tcp_client.h>

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

//headers in boost
#include <boost/thread.hpp>

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
            ros::param::param<int>(ros::this_node::getName()+"/mode", target, ALL);
            ros::param::param<int>(ros::this_node::getName()+"/mode", mode, REMOTE_OPERATED);
            ros::param::param<int>(ros::this_node::getName()+"/timeout", timeout, 30);
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
     * 
     * -1 < left_thruster_cmd < 1
     * 
     *  -1 < right_thruster_cmd < 1
     */
    void motor_command_callback_(std_msgs::Float64MultiArray msg);
    /**
     * @brief ROS callback function for /fix (message type : sensor_msgs/NavSatFix) topic.
     * 
     * @param msg ROS message
     */
    void fix_callback_(sensor_msgs::NavSatFix msg);
    /**
     * @brief send command to motor_driver or simulation.
     * 
     */
    void send_command_();
    /**
     * @brief publish heartbeat message to /heartbeat ROS topic
     * 
     */
    void publish_heartbeat_();
    ros::NodeHandle nh_;
    ros::Subscriber motor_command_sub_;
    ros::Subscriber fix_sub_;
    ros::Publisher usv_drive_cmd_pub_;
    ros::Publisher left_thrust_joint_pub_;
    ros::Publisher right_thrust_joint_pub_;
    ros::Publisher heartbeat_pub_;
    sensor_msgs::Joy last_joy_cmd_;
    std_msgs::Float64MultiArray last_motor_cmd_msg_;
    sensor_msgs::NavSatFix last_fix_msg_;
    const parameters params_;
    enum targets_{ALL=0,SIMULATION=1,HARDWARE=2};
    enum modes_{REMOTE_OPERATED=0,AUTONOMOUS=1,EMERGENCY=2};
    enum controllers_{dualshock4=0};
};

#endif //ROBOTX_HARDWARE_INTERFACE_H_INCLUDEDE