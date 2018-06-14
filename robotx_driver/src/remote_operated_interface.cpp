//headers in this package
#include <remote_operated_interface.h>

remote_operated_interface::remote_operated_interface
    (std::function<void(int)> set_action_mode_function, 
    std::function<void(std_msgs::Float64MultiArray motor_command)> send_motor_command) 
    : params_(remote_operated_interface::parameters())
{
    set_action_mode_function_ = set_action_mode_function;
    send_motor_command_ = send_motor_command;
    action_mode_signal_.connect(set_action_mode_function_);
    send_motor_command_signal_.connect(send_motor_command_);
    joy_sub_ = nh_.subscribe("/joy", 1, &remote_operated_interface::joy_callback_, this);
}

remote_operated_interface::~remote_operated_interface()
{

}

void remote_operated_interface::joy_callback_(sensor_msgs::Joy msg)
{
    last_joy_cmd_ = msg;
    return;
}