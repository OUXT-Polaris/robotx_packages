#include <heartbeat_publisher.h>

// hearers in stl
#include <stdio.h>
#include <string.h>

heartbeat_publisher::heartbeat_publisher() {
  message_recieved_ = false;
  nh_.param<std::string>(ros::this_node::getName() + "/ip_address", ip_address_, "127.0.0.1");
  nh_.param<int>(ros::this_node::getName() + "/port", port_, 31400);
  nh_.param<double>(ros::this_node::getName() + "/publish_rate", publish_rate_, 1);
  client_ = new tcp_client(io_service_, ip_address_, port_);
  io_service_.run();
  connection_status_pub_ = nh_.advertise<robotx_msgs::TechnicalDirectorNetworkStatus>(
      ros::this_node::getName() + "/connection_status", 1);
  tcp_thread = boost::thread(&heartbeat_publisher::publish_heartbeat_message, this);
  heartbeat_sub_ = nh_.subscribe(ros::this_node::getName() + "/heartbeat", 1,
                                 &heartbeat_publisher::heartbeat_callback, this);
}

heartbeat_publisher::~heartbeat_publisher() {}

void heartbeat_publisher::publish_heartbeat_message() {
  ros::Rate loop_rate(publish_rate_);
  while (ros::ok()) {
    publish_connection_status_message();
    mtx_.lock();
    if (message_recieved_ == true) {
      client_->send(tcp_send_msg_);
    }
    mtx_.unlock();
    loop_rate.sleep();
  }
}

void heartbeat_publisher::heartbeat_callback(const robotx_msgs::Heartbeat::ConstPtr &msg) {
  message_recieved_ = true;
  mtx_.lock();
  heartbeat_msg_ = *msg;
  update_heartbeat_message();
  mtx_.unlock();
}

std::string heartbeat_publisher::generate_checksum(const char *data) {
  int crc = 0;
  int i;
  // the first $ sign and the last two bytes of original CRC + the * sign
  for (i = 1; i < strlen(data) - 3; i++) {
    crc ^= data[i];
  }
  std::string checksum = "";
  std::stringstream ss;
  ss << std::hex << crc;
  std::dec;
  return checksum;
}

void heartbeat_publisher::update_heartbeat_message() {
  tcp_send_msg_ = "RXHRT,";
  tcp_send_msg_ = tcp_send_msg_ + heartbeat_msg_.utc_time_hh + heartbeat_msg_.utc_time_mm +
                  heartbeat_msg_.utc_time_ss + ",";
  tcp_send_msg_ = tcp_send_msg_ + std::to_string(heartbeat_msg_.latitude) + ",";
  if (heartbeat_msg_.north_or_south == heartbeat_msg_.NORTH) {
    tcp_send_msg_ = tcp_send_msg_ + "N,";
  } else {
    tcp_send_msg_ = tcp_send_msg_ + "S,";
  }
  tcp_send_msg_ = tcp_send_msg_ + std::to_string(heartbeat_msg_.longitude) + ",";
  if (heartbeat_msg_.east_or_west == heartbeat_msg_.EAST) {
    tcp_send_msg_ = tcp_send_msg_ + "E,";
  } else {
    tcp_send_msg_ = tcp_send_msg_ + "W,";
  }
  tcp_send_msg_ = tcp_send_msg_ + heartbeat_msg_.team_id;
  tcp_send_msg_ = tcp_send_msg_ + std::to_string(heartbeat_msg_.vehicle_mode) + ",";
  tcp_send_msg_ = tcp_send_msg_ + std::to_string(heartbeat_msg_.current_task_number);
  tcp_send_msg_ = "$" + tcp_send_msg_ + "*" + generate_checksum(tcp_send_msg_.c_str());
}

void heartbeat_publisher::publish_connection_status_message() {
  bool connection_status = client_->get_connection_status();
  robotx_msgs::TechnicalDirectorNetworkStatus connection_status_msg;
  if (connection_status == true) {
    connection_status_msg.status = connection_status_msg.CONNECTED;
  } else {
    connection_status_msg.status = connection_status_msg.CONNECTION_LOST;
  }
  connection_status_msg.port = port_;
  connection_status_msg.ip_address = ip_address_;
  connection_status_pub_.publish(connection_status_msg);
}
