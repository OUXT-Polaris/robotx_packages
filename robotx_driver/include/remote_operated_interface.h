#ifndef REMOTE_OPERATED_INTERFACE_H_INCLUDED
#define REMOTE_OPERATED_INTERFACE_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

//headers in STL
#include <functional>

//headers in boost
#include <boost/system/error_code.hpp>
#include <boost/signals2.hpp>
#include <boost/asio.hpp>

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
     * @param set_action_mode_function callback function for set_action_mode
     */
    remote_operated_interface(std::function<void(int)> set_action_mode_function);
    /**
     * @brief Destroy the remote operated interface object
     * 
     */
    ~remote_operated_interface();
private:
    /**
     * @brief parameters for remote_operated_interface class
     * 
     */
    const parameters params_;
    /**
     * @brief callback function for set_action_mode
     * 
     */
    std::function<void(int)> set_action_mode_function_;
    boost::signals2::signal<void (int)> signal_;
};

#endif  //REMOTE_OPERATED_INTERFACE_H_INCLUDED