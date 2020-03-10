#include "socket_tello.h"

#include <opencv2/highgui.hpp>
extern "C" 
{
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/mem.h>
#include <libswscale/swscale.h>
}

using namespace std;

VideoSocket::VideoSocket(unsigned short video_port, ros::Publisher pub) :
  TelloSocket(video_port), camera_pub(pub)
{
  packet_buffer = std::vector<unsigned char>(2048);
  buffer_list = std::vector<unsigned char>(65536);

  avcodec_register_all();

  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) throw std::runtime_error("Error finding decoder");

  context = avcodec_alloc_context3(codec);
  if (!context) throw std::runtime_error("Error allocating context");

  if(codec->capabilities & CODEC_CAP_TRUNCATED) context->flags |= CODEC_FLAG_TRUNCATED;

  if (avcodec_open2(context, codec, nullptr) < 0) throw std::runtime_error("Error opening context");

  parser = av_parser_init(AV_CODEC_ID_H264);
  if (!parser) throw std::runtime_error("Error initiating parser");

  frame = av_frame_alloc();
  if (!frame) throw std::runtime_error("Error allocating frame");

  pkt = new AVPacket;
  if (!pkt) throw std::runtime_error("Error allocating packet");

  av_init_packet(pkt);

  rgb_frame = av_frame_alloc();
  if (!rgb_frame) throw std::runtime_error("Error allocating RGB frame");
  rgb_context = nullptr;

  listen();
}

VideoSocket::~VideoSocket()
{
  av_parser_close(parser);
  avcodec_close(context);
  av_free(context);
  av_frame_free(&frame);
  delete pkt;
  sws_freeContext(rgb_context);
  av_frame_free(&rgb_frame);
}

void VideoSocket::process_packet(size_t size)
{
  if (next_buffer + size >= buffer_list.size()) 
  {
    next_buffer = 0;
    buffer_list_size = 0;
    return;
  }

  std::copy(packet_buffer.begin(), packet_buffer.begin() + size, buffer_list.begin() + next_buffer);
  next_buffer += size;
  buffer_list_size++;

  if (size < 1460) 
  {
    decode_frames();

    next_buffer = 0;
    buffer_list_size = 0;
  }

  return;
}

void VideoSocket::decode_frames()
{
  size_t next = 0;

  try 
  {
    while (next < next_buffer) 
    {
      // Parse H264
      ssize_t consumed = av_parser_parse2(parser, context, &pkt->data, &pkt->size, buffer_list.data() + next, next_buffer - next, 0, 0, AV_NOPTS_VALUE);

      if (pkt->size > 0) 
      {
        int got_picture = 0;
        int nread = avcodec_decode_video2(context, frame, &got_picture, pkt);
        if (nread < 0 || got_picture == 0) throw std::runtime_error("Error decoding frame");
        const AVFrame &frame_const = *frame;

        // YUV420P to BGR24
        int size = avpicture_fill((AVPicture*)rgb_frame, nullptr, AV_PIX_FMT_BGR24, frame_const.width, frame_const.height);
        unsigned char bgr24[size];

        rgb_context = sws_getCachedContext(rgb_context, frame_const.width, frame_const.height, (AVPixelFormat)frame_const.format, 
                                frame_const.width, frame_const.height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!rgb_context) throw std::runtime_error("Error allocating context");

        avpicture_fill((AVPicture*)rgb_frame, bgr24, AV_PIX_FMT_BGR24, frame_const.width, frame_const.height);
        sws_scale(rgb_context, frame_const.data, frame_const.linesize, 0, frame_const.height, rgb_frame->data, rgb_frame->linesize);
        rgb_frame->width = frame_const.width;
        rgb_frame->height = frame_const.height;

        cv::Mat mat{frame_const.height, frame_const.width, CV_8UC3, bgr24};

        std_msgs::Header header;
        ros::Time current_timestamp = ros::Time::now();
        header.stamp = current_timestamp;
        cv_bridge::CvImage image{header, sensor_msgs::image_encodings::BGR8, mat};
        
        camera_pub.publish(image.toImageMsg());
      }
      next += consumed;
    }
  }
  catch (std::runtime_error e) 
  {
  }
}
