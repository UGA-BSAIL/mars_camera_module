#include "camera_messenger.hpp"

#include <libcamera/pixel_format.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <chrono>
#include <cstdlib>
#include <limits>
#include <map>
#include <utility>

namespace libcamera_device {
namespace {

// Unfortunately, Boost has its own placeholders that conflict with the std
// ones, so we have to rename them.
const auto kStd1 = std::placeholders::_1;
const auto kStd2 = std::placeholders::_2;
const auto kStd3 = std::placeholders::_3;
const auto kStd4 = std::placeholders::_4;

// Maps LibCamera pixel formats to ROS pixel formats.
const std::map<libcamera::PixelFormat, std::string> kPixelFormatToEncoding = {
    {libcamera::formats::YUV422, sensor_msgs::image_encodings::YUV422},
    {libcamera::formats::RGB888, sensor_msgs::image_encodings::BGR8},
};

// Timeout to use when waiting for a frame before we consider the camera
// stalled.
const std::chrono::seconds kCameraTimeout(1000);

/**
 * @brief Converts a kernel timestamp to ROS time.
 * @param timestamp_us The kernel timestamp, in uS.
 * @return The ROS timestamp, in uS.
 */
uint64_t KernelToRosClock(uint32_t timestamp_us) {
  // Figure out the conversion.
  const auto kCurrentRosTime = ros::Time::now();
  const auto kCurrentKernelTime = std::chrono::steady_clock::now();
  const auto kOffset = std::chrono::nanoseconds{kCurrentRosTime.toNSec()} -
                       kCurrentKernelTime.time_since_epoch();
  const auto kUSecOffset =
      std::chrono::duration_cast<std::chrono::microseconds>(kOffset);

  // One of the joys of Libcamera is that it provides the timestamp as a 32-bit
  // integer, which means that it wraps after the camera has been running for
  // more than an hour or so. We handle this by using the kernel time to compute
  // the number of times it has wrapped.
  constexpr uint32_t kMaxSensorTime = std::numeric_limits<uint32_t>::max();
  const uint64_t kNumWraps =
      std::chrono::duration_cast<std::chrono::microseconds>(
          kCurrentKernelTime.time_since_epoch())
          .count() /
      kMaxSensorTime;
  const uint64_t kUnwrappedSensorTime =
      kNumWraps * kMaxSensorTime + static_cast<uint64_t>(timestamp_us);

  return kUnwrappedSensorTime + kUSecOffset.count();
}

}  // namespace

CameraMessenger::CameraMessenger(std::unique_ptr<RPiCamEncoder> &&camera_app,
                                 std::string frame_id,
                                 const VideoOptions &options)
    : camera_app_(std::move(camera_app)),
      frame_id_(std::move(frame_id)),
      on_message_ready_([](const sensor_msgs::Image &) {
        // Default callback does nothing, but logs a warning.
        ROS_WARN_STREAM("Got a camera message, but no callback is registered.");
      }) {
  ConfigureOptions(options);
}

CameraMessenger::~CameraMessenger() {
  // Make sure the camera is stopped.
  Stop();
}

void CameraMessenger::TranslateEncoded(void *buffer, size_t buffer_size,
                                       int64_t timestamp_us, uint32_t) {
  // Create the message for this image.
  sensor_msgs::Image message;

  // Sensor timestamps are from the kernel clock, but we want them relative to
  // the wall clock.
  const uint64_t kWallTimestampUs = KernelToRosClock(timestamp_us);
  message.header.stamp.sec = kWallTimestampUs / 1000000;
  message.header.stamp.nsec = (kWallTimestampUs % 1000000) * 1000;
  message.header.seq = message_sequence_++;
  message.header.frame_id = frame_id_;

  message.height = stream_info_.height;
  message.width = stream_info_.width;
  message.step = stream_info_.stride;
  message.is_bigendian = false;
  message.encoding = ros_pixel_format_;

  // I don't get why people still use void pointers in the Year of Our Lord
  // 2022...
  const uint8_t *byte_buffer = static_cast<uint8_t *>(buffer);
  // Copy raw image data.
  message.data.assign(byte_buffer, byte_buffer + buffer_size);

  // WaitForFrame the callback with the new message.
  on_message_ready_(message);
}

void CameraMessenger::SetMessageReadyCallback(
    const CameraMessenger::MessageReadyCallback &callback) {
  ROS_DEBUG_STREAM("Setting new callback for camera messages.");
  on_message_ready_ = callback;
}

void CameraMessenger::Start() {
  if (camera_running_) {
    // Already running.
    return;
  }
  camera_running_ = true;

  ROS_INFO_STREAM("Starting camera.");

  // Set up the callback.
  camera_app_->SetEncodeOutputReadyCallback(std::bind(
      &CameraMessenger::TranslateEncoded, this, kStd1, kStd2, kStd3, kStd4));

  ROS_DEBUG_STREAM("Opening camera.");
  camera_app_->OpenCamera();
  camera_app_->ConfigureVideo(RPiCamEncoder::FLAG_VIDEO_NONE);

  // Stream info isn't available until after the camera is configured.
  UpdateStreamInfo();

  camera_app_->StartEncoder();
  camera_app_->StartCamera();
}

void CameraMessenger::Stop() {
  if (!camera_running_) {
    // Camera already stopped.
    return;
  }
  camera_running_ = false;

  ROS_INFO_STREAM("Stopping camera.");
  camera_app_->StopCamera();
  camera_app_->StopEncoder();
  camera_app_->Teardown();
  camera_app_->CloseCamera();
}

bool CameraMessenger::WaitForFrame() {
  if (!camera_running_) {
    return false;
  }

  RPiCamEncoder::Msg message = camera_app_->Wait(kCameraTimeout);
  if (message.type == RPiCamEncoder::MsgType::Timeout) {
    ROS_FATAL_STREAM(
        "Timed out while waiting for a frame. This is either a hardware issue, "
        "or a bug in libcamera.");
    // It's not clear whether this is recoverable. Probably the best thing to do
    // is bail out and hove systemd restart the whole node.
    abort();
  } else if (message.type == RPiCamEncoder::MsgType::Quit) {
    ROS_INFO_STREAM("Got LibCamera quit request.");
    return false;
  }
  ROS_FATAL_STREAM_COND(
      message.type != RPiCamEncoder::MsgType::RequestComplete,
      "Got unrecognized message type " << static_cast<uint32_t>(message.type)
                                       << " from LibCamera!");

  auto &completed_request = std::get<CompletedRequestPtr>(message.payload);
  camera_app_->EncodeBuffer(completed_request, camera_app_->VideoStream());

  return true;
}

void CameraMessenger::UpdateStreamInfo() {
  stream_info_ = camera_app_->GetStreamInfo(camera_app_->VideoStream());

  // Set the pixel format correctly.
  const auto encoding = kPixelFormatToEncoding.find(stream_info_.pixel_format);
  ROS_FATAL_STREAM_COND(encoding == kPixelFormatToEncoding.end(),
                        "Got pixel format " << stream_info_.pixel_format
                                            << ", which is not supported.");
  ros_pixel_format_ = encoding->second;
}

void CameraMessenger::ConfigureOptions(const VideoOptions &new_options) {
  // Copy the specified options to the camera app.
  auto *options = camera_app_->GetOptions();

  options->nopreview = new_options.nopreview;
  options->denoise = new_options.denoise;
  options->codec = new_options.codec;
  options->ev = new_options.ev;
  options->brightness = new_options.brightness;
  options->contrast = new_options.contrast;
  options->saturation = new_options.saturation;
  options->sharpness = new_options.sharpness;
  options->framerate = new_options.framerate;
  options->mode = new_options.mode;
  options->width = new_options.width;
  options->height = new_options.height;
  options->camera = new_options.camera;
}

}  // namespace libcamera_device
