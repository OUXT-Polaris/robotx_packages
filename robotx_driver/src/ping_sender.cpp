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
    XmlRpc::XmlRpcValue target_device_params = parameters["target_device"];
    for(auto param_itr = target_device_params.begin(); param_itr != target_device_params.end(); ++param_itr)
    {
        ip_table_.insert(std::make_pair(param_itr->first,device_info(param_itr->second["ip_address"],param_itr->second["device_id"])));
    }
}

ping_sender::~ping_sender()
{

}