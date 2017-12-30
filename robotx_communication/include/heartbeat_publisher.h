#ifndef HEARTBEAT_PUBLISHER_H_INCLUDED
#define HEARTBEAT_PUBLISHER_H_INCLUDED

//headers in this package
#include <tcp_client.h>
#include <robotx_msgs/Heartbeat.h>
#include <robotx_msgs/TechnicalDirectorNetworkStatus.h>

//headers in ros
#include <ros/ros.h>

//headers in boost
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

//headers in stl
#include <mutex>

class heartbeat_publisher
{
public:
  heartbeat_publisher();
  ~heartbeat_publisher();
private:
  boost::thread tcp_thread;
  void heartbeat_callback(const robotx_msgs::Heartbeat::ConstPtr& msg);
  void publish_connection_status_message();
  void publish_heartbeat_message();
  void update_heartbeat_message();
  std::string generate_checksum(const char *data);
  ros::NodeHandle nh_;
  //members for subscription
  ros::Subscriber heartbeat_sub_;
  robotx_msgs::Heartbeat heartbeat_msg_;
  std::string tcp_send_msg_;
  std::mutex mtx_;
  //members for publish connection status
  ros::Publisher connection_status_pub_;
  //parametes for heartbeat connection
  tcp_client* client_;
  boost::asio::io_service io_service_;
  std::string ip_address_;
  int port_;
  double publish_rate_;
  //flags
  volatile bool message_recieved_;
};

#endif  //HEARTBEAT_PUBLISHER_H_INCLUDED
