#include "socket_tello.h"

using namespace std;

StateSocket::StateSocket(unsigned short data_port) : TelloSocket(data_port)
{
  packet_buffer = std::vector<unsigned char>(1024);
}

std::string StateSocket::listen_once()
{
  size_t size = socket.receive(boost::asio::buffer(packet_buffer));

  std::string raw(packet_buffer.begin(), packet_buffer.begin() + size);
  return raw;
}

void StateSocket::process_packet(size_t size)
{
  std::string raw(packet_buffer.begin(), packet_buffer.begin() + size);

  return;
}
