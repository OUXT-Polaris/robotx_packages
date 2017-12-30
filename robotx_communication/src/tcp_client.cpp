//headers in this package
#include <tcp_client.h>

//headers in ROS
#include <ros/ros.h>

tcp_client::tcp_client(boost::asio::io_service& io_service,std::string ip_address,int port):io_service_(io_service),socket_(io_service)
{
  connection_status_ = false;
  ip_address_ = ip_address;
  port_ = port;
  connect();
}

tcp_client::~tcp_client()
{

}

void tcp_client::connect()
{
  socket_.async_connect(
    boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip_address_), port_),
    boost::bind(&tcp_client::on_connect, this, boost::asio::placeholders::error));
}

//callback function on connect
void tcp_client::on_connect(const boost::system::error_code& error)
{
  if(error)
  {
    connection_status_ = false;
    ROS_ERROR_STREAM("connect failed : " << error.message());
  }
  else
  {
    connection_status_ = true;
    ROS_INFO_STREAM("connected");
  }
}

void tcp_client::send(std::string data)
{
  //send data by using tcp/ip protocol
  boost::asio::async_write(
    socket_,
    boost::asio::buffer(data),
    boost::bind(&tcp_client::on_send, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

//callback function for sending data
void tcp_client::on_send(const boost::system::error_code& error, size_t bytes_transferred)
{
  if(error)
  {
    std::cout << "send failed: " << error.message() << std::endl;
  }
  else
  {
    std::cout << "send correct!" << std::endl;
  }
}
