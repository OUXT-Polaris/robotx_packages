#ifndef TCP_CLIENT_H_INCLUDED
#define TCP_CLIENT_H_INCLUDED

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

/**
*  @brief async TCP/IP client class using boost/asio
*  @details https://boostjp.github.io/tips/network/tcp.html
*  @author Masaya Kataoka
*  @date 2018.06.09
*/
class tcp_client
{
public:
  /**
  * @brief constructor 
  * 
  * @param io_service io_service for TCP/IP clienet.
  * 
  * @param ip_address ip_address of target tcp/ip server. ex. "127.0.0.1"
  * 
  * @param port of target tcp/ip server. ex. 8000
  */
  tcp_client(boost::asio::io_service& io_service,std::string ip_address,int port);
  /**
  * @brief destructor
  */
  ~tcp_client();
  /**
  * @brief send string data
  * 
  * @param data content for sending
  */
  void send(std::string data);
  /**
   * @brief Get the connection status object
   * 
   * @return true : connecsion success
   * @return false : connection failed
   */
  inline bool get_connection_status(){return connection_status_;}
private:
  /**
   * @brief function for start connecting to TCP/IP server.
   * 
   */
  void connect();
  /**
   * @brief callback function which was called when the client make a success of connection.
   * 
   * @param error error code for TCP/IP connection.
   */
  void on_connect(const boost::system::error_code& error);
  /**
   * @brief callback function which was called when the client send a message to the TCP/IP server.
   * 
   * @param error error code for data transfer.
   * @param bytes_transferred number of bytes transferrd.
   */
  void on_send(const boost::system::error_code& error, size_t bytes_transferred);
  /**
   * @brief io_service for the connection.
   * 
   */
  boost::asio::io_service& io_service_;
  /**
   * @brief TCP/IP socket
   * 
   */
  boost::asio::ip::tcp::socket socket_;
  /**
   * @brief parameter for target IP address.
   * 
   */
  std::string ip_address_;
  /**
   * @brief parameter for target network port.
   * 
   */
  int port_;
  /**
   * @brief parameters for connection status.
   * 
   * @sa tcp_client::get_connection_status()
   * 
   */
  volatile bool connection_status_;
};
#endif  //TCP_CLIENT_H_INCLUDED