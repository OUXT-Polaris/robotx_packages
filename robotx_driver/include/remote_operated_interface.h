#ifndef REMOTE_OPERATED_INTERFACE_H_INCLUDED
#define REMOTE_OPERATED_INTERFACE_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

class remote_operated_interface
{
public:
    struct parameters
    {
        enum controllers {DUALSHOCK4=0};
        int controller_type;
        parameters()
        {
            ros::param::param<int>(ros::this_node::getName()+"/controller_type", controller_type, DUALSHOCK4);
        }
    };
    remote_operated_interface();
    ~remote_operated_interface();
private:
    const parameters params_;
};

#endif  //REMOTE_OPERATED_INTERFACE_H_INCLUDED