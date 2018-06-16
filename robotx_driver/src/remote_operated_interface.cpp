//headers in this package
#include <remote_operated_interface.h>
#include <robotx_hardware_interface.h>

remote_operated_interface::remote_operated_interface
    (std::function<void(int)> set_action_mode_function, 
    std::function<void(std_msgs::Float64MultiArray motor_command)> send_motor_command) 
    : params_()
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
    std_msgs::Float64MultiArray motor_command_msg;
    if(params_.controller_type == params_.DUALSHOCK4)
    {
        if(last_joy_cmd_.buttons[12] == 1)
            action_mode_signal_(robotx_hardware_interface::parameters::REMOTE_OPERATED);
        if(last_joy_cmd_.buttons[13] == 1)
            action_mode_signal_(robotx_hardware_interface::parameters::AUTONOMOUS);
    }
    return;
}