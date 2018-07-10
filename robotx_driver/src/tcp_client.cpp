// headers in this package
#include <tcp_client.h>

// headers in ROS
#include <ros/ros.h>

// headers in STL
#include <array>
#include <chrono>

tcp_client::tcp_client(boost::asio::io_service &io_service, std::string ip_address, int port)
    : io_service_(io_service), socket_(io_service), timer_(io_service), is_canceled_(false) {
  connection_status_ = false;
  ip_address_ = ip_address;
  port_ = port;
  timeout_ = 30;
  connect();
}

tcp_client::tcp_client(boost::asio::io_service &io_service, std::string ip_address, int port, int timeout)
    : io_service_(io_service), socket_(io_service), timer_(io_service), is_canceled_(false) {
  connection_status_ = false;
  ip_address_ = ip_address;
  port_ = port;
  timeout_ = timeout;
  connect();
}

tcp_client::~tcp_client() {}

void tcp_client::connect() {
  socket_.async_connect(
      boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip_address_), port_),
      boost::bind(&tcp_client::on_connect, this, boost::asio::placeholders::error));
}

void tcp_client::on_timer(const boost::system::error_code &error) {
  if (!error && !is_canceled_) {
    socket_.cancel();
  }
}

// callback function on connect
void tcp_client::on_connect(const boost::system::error_code &error) {
  if (error) {
    connection_status_ = false;
    ROS_ERROR_STREAM("connect failed : " << error.message() << "(" << ip_address_ << ":" << port_ << ")");
  } else {
    connection_status_ = true;
    ROS_INFO_STREAM("connected"
                    << "(" << ip_address_ << ":" << port_ << ")");
  }
  start_receive();
}

void tcp_client::on_receive(const boost::system::error_code &error, size_t bytes_transferred) {
  if (error == boost::asio::error::operation_aborted) {
    ROS_ERROR_STREAM("timeout"
                     << "(" << ip_address_ << ":" << port_ << ")");
  } else {
    timer_.cancel();
    is_canceled_ = true;
    if (error) {
      ROS_ERROR_STREAM(error.message());
    }
  }
}

void tcp_client::send(double data) {
  // send data by using tcp/ip protocol
  std::array<double, 1> data_arr;
  data_arr[0] = data;
  boost::asio::async_write(socket_, boost::asio::buffer(data_arr),
                           boost::bind(&tcp_client::on_send, this, boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));
}

void tcp_client::send(std::string data) {
  // send data by using tcp/ip protocol
  boost::asio::async_write(socket_, boost::asio::buffer(data),
                           boost::bind(&tcp_client::on_send, this, boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));
}

// callback function for sending data
void tcp_client::on_send(const boost::system::error_code &error, size_t bytes_transferred) {
  if (error) {
    std::cout << "send failed: " << error.message() << std::endl;
  } else {
    std::cout << "send correct!" << std::endl;
  }
}

void tcp_client::start_receive() {
  boost::asio::async_read(socket_, receive_buff_, boost::asio::transfer_all(),
                          boost::bind(&tcp_client::on_receive, this, boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));
  timer_.expires_from_now(std::chrono::seconds(timeout_));
  timer_.async_wait(boost::bind(&tcp_client::on_timer, this, _1));
}
