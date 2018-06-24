#ifndef PING_SENDER_H_INCLUDED
#define PING_SENDER_H_INCLUDED

//headers in STL
#include <map>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <resolv.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>

//headers in ROS
#include <ros/ros.h>

#define PACKETSIZE  64
struct packet
{
    struct icmphdr hdr;
    char msg[PACKETSIZE-sizeof(struct icmphdr)];
};

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
};
#endif  //PING_SENDER_H_INCLUDED