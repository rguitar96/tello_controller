#include "socket_tello.h"

using namespace std;

void TelloSocket::listen()
{
  thread = std::thread(
  [this]()
  {
    while (true) {
      size_t size = socket.receive(boost::asio::buffer(packet_buffer));
      process_packet(size);
    }
  });
  return;
}
