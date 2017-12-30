#ifndef TCP_CLIENT_H_INCLUDED
#define TCP_CLIENT_H_INCLUDED

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

class tcp_client
{
public:
  tcp_client(boost::asio::io_service& io_service,std::string ip_address,int port);
  ~tcp_client();
  void send(std::string data);
private:
  void connect();
  void on_connect(const boost::system::error_code& error);
  void on_send(const boost::system::error_code& error, size_t bytes_transferred);
  boost::asio::io_service& io_service_;
  boost::asio::ip::tcp::socket socket_;
  std::string ip_address_;
  int port_;
};
#endif  //TCP_CLIENT_H_INCLUDED
