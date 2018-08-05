#include <tcp_server.h>

tcp_server::tcp_server(int server_port, boost::asio::io_service& io_service)
    : io_service_(io_service),
      port(server_port),
      socket_(io_service),
      acceptor_(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), server_port)) {}

tcp_server::~tcp_server() { std::cout << "close connection" << std::endl; }

void tcp_server::start() {
  std::cout << "start accept connection" << std::endl;
  start_accept_();
}

void tcp_server::start_accept_() {
  acceptor_.async_accept(socket_,
                         boost::bind(&tcp_server::on_accept_, this, boost::asio::placeholders::error));
}

void tcp_server::on_accept_(const boost::system::error_code& error) {
  if (error) {
    std::cout << "accept failed: " << error.message() << std::endl;
    return;
  }
  start_receive_();
}

void tcp_server::start_receive_() {
  boost::asio::async_read(socket_, receive_buff_, boost::asio::transfer_all(),
                          boost::bind(&tcp_server::on_receive_, this, boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));
}

void tcp_server::on_receive_(const boost::system::error_code& error, size_t bytes_transferred) {
  if (error && error != boost::asio::error::eof) {
    std::cout << "receive failed: " << error.message() << std::endl;
  } else {
    const char* data = boost::asio::buffer_cast<const char*>(receive_buff_.data());
    std::cout << data << std::endl;

    receive_buff_.consume(receive_buff_.size());
  }
}