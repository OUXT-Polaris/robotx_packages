#include <tcp_server.h>
#include <boost/asio.hpp>

int main() {
  boost::asio::io_service io_service;
  tcp_server server(8080, io_service);
  server.start();
  io_service.run();
  return 0;
}