#ifndef TCP_SERVER_H_INCLUDED
#define TCP_SERVER_H_INCLUDED

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>

class tcp_server {
 public:
  tcp_server(int server_port, boost::asio::io_service& io_service);
  ~tcp_server();
  const int port;
  void start();

 private:
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::io_service& io_service_;
  boost::asio::streambuf receive_buff_;
  void start_accept_();
  void on_accept_(const boost::system::error_code& error);
  void start_receive_();
  void on_receive_(const boost::system::error_code& error, size_t bytes_transferred);
};
#endif  // TCP_SERVER_H_INCLUDED