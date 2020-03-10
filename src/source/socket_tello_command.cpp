#include "socket_tello.h"

using namespace std;

  CommandSocket::CommandSocket(std::string drone_ip,
                               unsigned short drone_port, unsigned short command_port) :
    TelloSocket(command_port),
    endpoint(boost::asio::ip::address_v4::from_string(drone_ip), drone_port)
  {
    packet_buffer = std::vector<unsigned char>(1024);
  }

  void CommandSocket::send_command(std::string command)
  {
    socket.send_to(boost::asio::buffer(command), endpoint);
  }
  
  void CommandSocket::process_packet(size_t size)
  {
    return;
  }

