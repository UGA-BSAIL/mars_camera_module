/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "ffmpeg_image_transport/ffmpeg_decoder.h"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/Image.h>
#include <string>
#include <unordered_map>

namespace ffmpeg_image_transport {
namespace {

/**
 * @brief Gets the description for an FFMPEG error.
 * @param av_return The error number from FFMPEG.
 * @return The error description.
 */
std::string DescribeAvError(int av_return) {
  char error_c_str[AV_ERROR_MAX_STRING_SIZE];
  av_strerror(av_return, error_c_str, AV_ERROR_MAX_STRING_SIZE);
  return {error_c_str};
}

} // namespace

FFMPEGDecoder::FFMPEGDecoder() {
  codecMap_["h264_nvenc"] = {"h264"};
  codecMap_["h264_v4l2m2m"] = {"h264"};
  codecMap_["libx264"] = {"h264"};
  codecMap_["hevc_nvenc"] = {"hevc_cuvid", "hevc"};
}

FFMPEGDecoder::~FFMPEGDecoder() { reset(); }

void FFMPEGDecoder::reset() {
  if (codecContext_) {
    avcodec_close(codecContext_);
    av_free(codecContext_);
    codecContext_ = NULL;
  }
  if (swsContext_) {
    sws_freeContext(swsContext_);
    swsContext_ = NULL;
  }
  if (hwDeviceContext_) {
    av_buffer_unref(&hwDeviceContext_);
  }
  av_free(decodedFrame_);
  decodedFrame_ = NULL;
  av_free(cpuFrame_);
  cpuFrame_ = NULL;
  av_free(colorFrame_);
  colorFrame_ = NULL;
}

bool FFMPEGDecoder::initialize(const FFMPEGPacket::ConstPtr &msg,
                               Callback callback,
                               const std::string &codecName) {
  callback_ = callback;
  std::string cname = codecName;
  std::vector<std::string> codecs;
  if (cname.empty()) {
    // try and find the right codec from the map
    const auto it = codecMap_.find(msg->encoding);
    if (it == codecMap_.end()) {
      ROS_ERROR_STREAM("message has unknown encoding: " << msg->encoding);
      return (false);
    }
    cname = msg->encoding;
    codecs = it->second;
  } else {
    codecs.push_back(codecName);
  }
  encoding_ = msg->encoding;
  return (initDecoder(msg->img_width, msg->img_height, cname, codecs));
}

bool FFMPEGDecoder::initDecoder(int width, int height,
                                const std::string &codecName,
                                const std::vector<std::string> &codecs) {
  std::string codecUsed = "NO_CODEC_FOUND";
  try {
    const AVCodec *codec = NULL;
    for (const auto &c : codecs) {
      codec = avcodec_find_decoder_by_name(c.c_str());
      if (!codec) {
        ROS_WARN_STREAM("no codec " << c << " found!");
        continue;
      }
      codecContext_ = avcodec_alloc_context3(codec);
      if (!codecContext_) {
        ROS_WARN_STREAM("alloc context failed for " + codecName);
        codec = NULL;
        continue;
      }
      av_opt_set_int(codecContext_, "refcounted_frames", 1, 0);

      hwPixFormat_ = AV_PIX_FMT_NONE;
      codecContext_->width = width;
      codecContext_->height = height;
      codecContext_->pix_fmt = AV_PIX_FMT_YUV420P;

      if (avcodec_open2(codecContext_, codec, NULL) < 0) {
        ROS_WARN_STREAM("open context failed for " + codecName);
        av_free(codecContext_);
        codecContext_ = NULL;
        codec = NULL;
        continue;
      }
      codecUsed = c;
      break;
    }
    if (!codec)
      throw(std::runtime_error("cannot open codec " + codecName));

    decodedFrame_ = av_frame_alloc();
    cpuFrame_ = (hwPixFormat_ == AV_PIX_FMT_NONE) ? NULL : av_frame_alloc();
    colorFrame_ = av_frame_alloc();
    colorFrame_->width = width;
    colorFrame_->height = height;
    colorFrame_->format = AV_PIX_FMT_BGR24;

  } catch (const std::runtime_error &e) {
    ROS_ERROR_STREAM(e.what());
    reset();
    return (false);
  }
  if (codecName != codecUsed) {
    ROS_INFO_STREAM("message encoded with " << codecName << " decoded with "
                                            << codecUsed);
  } else {
    ROS_INFO_STREAM("decoding with " << codecUsed);
  }
  return (true);
}

bool FFMPEGDecoder::decodePacket(const FFMPEGPacket::ConstPtr &msg) {
  // Send the current packet.
  bool all_successful = true;
  all_successful = sendPacket(msg) && all_successful;

  // Receive any frames that are ready.
  while (all_successful) {
    all_successful = receiveFrame() && all_successful;
  }

  return all_successful;
}

bool FFMPEGDecoder::sendPacket(const ffmpeg_image_transport_msgs::FFMPEGPacket_<
                               std::allocator<void>>::ConstPtr &msg) {
  if (msg->encoding != encoding_) {
    ROS_ERROR_STREAM("cannot change encoding on the fly!!!");
    return false;
  }
  AVCodecContext *ctx = codecContext_;
  AVPacket packet;
  av_init_packet(&packet);
  av_new_packet(&packet, msg->data.size()); // will add some padding!
  memcpy(packet.data, &msg->data[0], msg->data.size());
  packet.pts = msg->pts;
  packet.dts = packet.pts;
  ptsToHeader_[packet.pts] = msg->header;
  int ret = avcodec_send_packet(ctx, &packet);
  if (ret != 0) {
    ROS_WARN_STREAM("send_packet failed for pts " << msg->pts << ": "
                                                  << DescribeAvError(ret));
    av_packet_unref(&packet);
    ptsToHeader_.erase(packet.pts);
    return false;
  }

  av_packet_unref(&packet);
  return true;
}

bool FFMPEGDecoder::receiveFrame() {
  ros::WallTime t0;
  if (measurePerformance_) {
    t0 = ros::WallTime::now();
  }

  AVCodecContext *ctx = codecContext_;
  int ret = avcodec_receive_frame(ctx, decodedFrame_);
  if (ret != 0) {
    ROS_ERROR_STREAM_COND(ret != AVERROR(EAGAIN),
                          "Failed to decode frame: " << DescribeAvError(ret));
    return false;
  }

  if (decodedFrame_->width != 0) {
    // convert image to something palatable
    if (!swsContext_) {
      swsContext_ = sws_getContext(
          ctx->width, ctx->height, (AVPixelFormat)decodedFrame_->format, // src
          ctx->width, ctx->height, (AVPixelFormat)colorFrame_->format,   // dest
          SWS_FAST_BILINEAR, NULL, NULL, NULL);
      if (!swsContext_) {
        ROS_FATAL("cannot allocate sws context!!!!");
        ros::shutdown();
        return false;
      }
    }
    // prepare the decoded message
    ImagePtr image(new sensor_msgs::Image());
    image->height = decodedFrame_->height;
    image->width = decodedFrame_->width;
    image->step = image->width * 3; // 3 bytes per pixel
    image->encoding = sensor_msgs::image_encodings::BGR8;
    image->data.resize(image->step * image->height);

    // bend the memory pointers in colorFrame to the right locations
    av_image_fill_arrays(colorFrame_->data, colorFrame_->linesize,
                         &(image->data[0]), (AVPixelFormat)colorFrame_->format,
                         colorFrame_->width, colorFrame_->height, 1);
    sws_scale(swsContext_, decodedFrame_->data, decodedFrame_->linesize,
              0,                                                      // src
              ctx->height, colorFrame_->data, colorFrame_->linesize); // dest
    auto it = ptsToHeader_.find(decodedFrame_->pts);
    if (it == ptsToHeader_.end()) {
      ROS_ERROR_STREAM("cannot find pts that matches " << decodedFrame_->pts);
    } else {
      image->header = it->second;
      ptsToHeader_.erase(it);
      callback_(image, decodedFrame_->key_frame == 1); // deliver callback
    }
  }

  if (measurePerformance_) {
    ros::WallTime t1 = ros::WallTime::now();
    double dt = (t1 - t0).toSec();
    tdiffTotal_.update(dt);
  }
  return true;
}

void FFMPEGDecoder::resetTimers() { tdiffTotal_.reset(); }

void FFMPEGDecoder::printTimers(const std::string &prefix) const {
  ROS_INFO_STREAM(prefix << " total decode: " << tdiffTotal_);
}

} // namespace ffmpeg_image_transport
