//headers in this package
#include <remote_operated_interface.h>

remote_operated_interface::remote_operated_interface(std::function<void(int)> set_action_mode_function) 
    : params_(remote_operated_interface::parameters())
{
    set_action_mode_function_ = set_action_mode_function;
    action_mode_signal_.connect(set_action_mode_function_);
}

remote_operated_interface::~remote_operated_interface()
{

}

void remote_operated_interface::joy_callback_(sensor_msgs::Joy msg)
{
    last_joy_cmd_ = msg;
    return;
}