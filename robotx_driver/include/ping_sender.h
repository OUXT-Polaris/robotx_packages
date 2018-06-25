#ifndef PING_SENDER_H_INCLUDED
#define PING_SENDER_H_INCLUDED

//headers in STL
#include <map>
#include <thread>
#include <stdlib.h>
#include <stdio.h>

//headers in ROS
#include <ros/ros.h>

struct device_info
{
    std::string ip_address;
    int device_id;
    device_info(std::string ip_address_,int device_id_)
    {
        ip_address = ip_address_;
        device_id = device_id_;
    }
};

/**
 * @brief ping sender class
 * 
 */
class ping_sender
{
public:
    ping_sender();
    ~ping_sender();
private:
    /**
     * @brief send ping to target ip_address
     * 
     * @param ip_address ip_adress
     * @return true succeed to send ping
     * @return false failed to send ping
     */
    bool ping_(std::string ip_address);
    /**
     * @brief ROS nodehandle
     * 
     */
    ros::NodeHandle nh_;
    /**
     * @brief ip tables
     * 
     */
    std::map<std::string,device_info> ip_table_;
    /**
     * @brief frequency of ping command
     * 
     */
    double frequency_;
    /**
     * @brief timeout of ping command (maximun = 0.5*1/frequency_) 
     * 
     */
    double timeout_;
};
#endif  //PING_SENDER_H_INCLUDED