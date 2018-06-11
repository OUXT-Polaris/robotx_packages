//headers in this package
#include <robotx_hardware_interface.h>
#include <robotx_msgs/UsvDrive.h>

robotx_hardware_interface::robotx_hardware_interface() : params_(robotx_hardware_interface::parameters())
{
    if(params_.target == ALL || params_.target == SIMULATION)
    {
        usv_drive_cmd_pub_ = nh_.advertise<robotx_msgs::UsvDrive>("/cmd_drive", 1);
        left_thrust_joint_pub_ = nh_.advertise<std_msgs::Float64>("/left_thruster_position_controller/command",1);
        right_thrust_joint_pub_ = nh_.advertise<std_msgs::Float64>("/right_thruster_position_controller/command",1);
        last_motor_cmd_msg_.data.resize(4);
        last_motor_cmd_msg_.data[0] = 0;
        last_motor_cmd_msg_.data[1] = 0;
        last_motor_cmd_msg_.data[2] = 0;
        last_motor_cmd_msg_.data[3] = 0;
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
    if(msg.data.size() == 4)
    {
        last_motor_cmd_msg_ = msg;  
    }
    return;
}

void robotx_hardware_interface::send_command_()
{
    if(params_.target == ALL || params_.target == SIMULATION)
    {
        robotx_msgs::UsvDrive usv_drive_msg;
        usv_drive_msg.left = last_motor_cmd_msg_.data[0];
        usv_drive_msg.right = last_motor_cmd_msg_.data[2];
        std_msgs::Float64 left_thrust_joint_cmd_;
        left_thrust_joint_cmd_.data = last_motor_cmd_msg_.data[1];
        std_msgs::Float64 right_thrust_joint_cmd_;
        right_thrust_joint_cmd_.data = last_motor_cmd_msg_.data[3];
        usv_drive_cmd_pub_.publish(usv_drive_msg);
        left_thrust_joint_pub_.publish(left_thrust_joint_cmd_);
        right_thrust_joint_pub_.publish(right_thrust_joint_cmd_);
    }
}