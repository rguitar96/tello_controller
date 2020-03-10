#ifndef TELLO_DRIVER_H
#define TELLO_DRIVER_H

#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"

#include <cstdlib>
#include <stdexcept>


#ifndef PIX_FMT_RGB24
#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24
#endif

#ifndef CODEC_CAP_TRUNCATED
#define CODEC_CAP_TRUNCATED AV_CODEC_CAP_TRUNCATED
#endif

#ifndef CODEC_FLAG_TRUNCATED
#define CODEC_FLAG_TRUNCATED AV_CODEC_FLAG_TRUNCATED
#endif

using boost::asio::ip::udp;

#define TELLO_CLIENT_ADDRESS "192.168.10.1"
#define TELLO_SERVER_ADDRESS "192.168.10.2"
#define TELLO_COMMAND_PORT 8889
#define PC_COMMAND_PORT 38065
#define TELLO_STATE_PORT 8890
#define TELLO_CAMERA_PORT 11111

class CommandSocket;
class StateSocket;
class VideoSocket;

struct AVCodecContext;
struct AVFrame;
struct AVCodec;
struct AVCodecParserContext;
struct SwsContext;
struct AVPacket;

class TelloSocket
{
public:
  TelloSocket(unsigned short port) :
    socket(io_service, udp::endpoint(udp::v4(), port)) {}

  void listen();

protected:
  virtual void process_packet(size_t size) = 0;

  boost::asio::io_service io_service;
  udp::socket socket;
  std::thread thread;
  std::vector<unsigned char> packet_buffer;
};

class StateSocket : public TelloSocket
{
public:
  StateSocket(unsigned short data_port);
  std::string listen_once();

private:
  void process_packet(size_t size) override;
};

class CommandSocket : public TelloSocket
{
public:
  CommandSocket(std::string drone_ip, unsigned short drone_port, unsigned short command_port);
  void send_command(std::string command);

private:
  void process_packet(size_t size) override;

  udp::endpoint endpoint;
};

class VideoSocket : public TelloSocket
{
  AVCodecContext        *context;
  AVFrame               *frame;
  AVCodec               *codec;
  AVCodecParserContext  *parser;
  AVPacket              *pkt;

  SwsContext *rgb_context;
  AVFrame *rgb_frame;

public:
  VideoSocket(unsigned short video_port, ros::Publisher pub);
  ~VideoSocket();

private:
  void process_packet(size_t size) override;
  void decode_frames();

  std::vector<unsigned char> buffer_list;
  int buffer_list_size = 0;
  size_t next_buffer = 0;

  ros::Publisher camera_pub;
};

#endif