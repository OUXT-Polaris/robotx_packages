//headers in this package
#include <remote_operated_interface.h>

remote_operated_interface::remote_operated_interface(std::function<void(int)> set_action_mode_function) 
    : params_(remote_operated_interface::parameters())
{
    set_action_mode_function_ = set_action_mode_function;
    signal_.connect(set_action_mode_function_);
}

remote_operated_interface::~remote_operated_interface()
{

}