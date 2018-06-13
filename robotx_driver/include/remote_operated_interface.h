#ifndef REMOTE_OPERATED_INTERFACE_H_INCLUDED
#define REMOTE_OPERATED_INTERFACE_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

/**
 * @brief interface for remote operation
 * 
 */
class remote_operated_interface
{
public:
    /**
     * @brief remote_operated_interface class parameters
     * 
     */
    struct parameters
    {
        enum controllers {DUALSHOCK4=0};
        int controller_type;
        /**
         * @brief Construct a new parameters objects
         * 
         */
        parameters()
        {
            ros::param::param<int>(ros::this_node::getName()+"/controller_type", controller_type, DUALSHOCK4);
        }
    };
    /**
     * @brief Construct a new remote operated interface object
     * 
     */
    remote_operated_interface();
    /**
     * @brief Destroy the remote operated interface object
     * 
     */
    ~remote_operated_interface();
private:
    const parameters params_;
};

#endif  //REMOTE_OPERATED_INTERFACE_H_INCLUDED