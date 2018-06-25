#include <ping_sender.h>

ping_sender::ping_sender()
{
    XmlRpc::XmlRpcValue parameters;
    nh_.getParam(ros::this_node::getName(), parameters);
    try
    {
        frequency_ = parameters["frequency"];
    }
    catch(...)
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " : failed to get frequency parameters, set 1.0");
        frequency_ = 1.0;
    }
    try
    {
        timeout_ = parameters["timeout"];
    }
    catch(...)
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " : failed to get timeout parameters, set 1.0");
        timeout_ = 1/frequency_*0.5;
    }
    if(timeout_ > 1/frequency_*0.5)
    {
        timeout_ = 1/frequency_*0.5;
    }
    XmlRpc::XmlRpcValue target_device_params = parameters["target_device"];
    for(auto param_itr = target_device_params.begin(); param_itr != target_device_params.end(); ++param_itr)
    {
        ip_table_.insert(std::make_pair(param_itr->first,device_info(param_itr->second["ip_address"],param_itr->second["device_id"])));
    }
    int num_target = ip_table_.size();
    std::vector<std::thread> sender_thread(num_target);
}

ping_sender::~ping_sender()
{

}

bool ping_sender::ping_(std::string ip_address)
{
    int ret;
    int state;
    std::string command = "ping -c 1 -w " + std::to_string(timeout_) + " " + ip_address;
    ret = system(command.c_str());
    if(WIFEXITED(ret))
    {
        state = WEXITSTATUS(ret);
    }
    else
    {
        state = -1;
    }
    if(state == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}