//headers in this package
#include <robotx_hardware_interface.h>
#include <robotx_msgs/UsvDrive.h>

robotx_hardware_interface::robotx_hardware_interface() : params_(robotx_hardware_interface::parameters())
{
    if(params_.target == all || params_.target == simulation )
    {
        usv_drive_cmd_pub_ = nh_.advertise<robotx_msgs::UsvDrive>("/cmd_drive", 1);
        left_thrust_joint_pub_ = nh_.advertise<std_msgs::Float64>("/left_thruster_position_controller/command",1);
        right_thrust_joint_pub_ = nh_.advertise<std_msgs::Float64>("/right_thruster_position_controller/command",1);
    }
}

robotx_hardware_interface::~robotx_hardware_interface()
{

}

void robotx_hardware_interface::joy_callback_(sensor_msgs::Joy msg)
{
    last_joy_cmd_ = msg;
    return;
}

//msg = [left_thruster_cmd left_thruster_joint_angle right_thruster_cmd right_thruster_joint_angle]
void robotx_hardware_interface::motor_command_callback_(std_msgs::Float64MultiArray msg)
{
    last_motor_cmd_msg_ = msg;
    return;
}