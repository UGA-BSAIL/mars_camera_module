#include "camera_messenger.hpp"

#include <hailort.h>
#include <libcamera/pixel_format.h>
#include <libcamera_device/Detection.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "libcamera/formats.h"
#include "post_processing_stages/hailo/hailo_postprocessing_stage.hpp"
#include "post_processing_stages/object_detect.hpp"

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
    {libcamera::formats::SRGGB16, sensor_msgs::image_encodings::BAYER_RGGB16},
};

// Timeout to use when waiting for a frame before we consider the camera
// stalled.
const std::chrono::seconds kCameraTimeout(10);

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

CameraMessenger::CameraMessenger(std::unique_ptr<RPiCamEncoder>&& camera_app,
                                 std::string frame_id,
                                 const VideoOptions& options)
    : camera_app_(std::move(camera_app)),
      frame_id_(std::move(frame_id)),
      on_message_ready_([](const sensor_msgs::Image&) {
        // Default callback does nothing, but logs a warning.
        ROS_WARN_STREAM("Got a camera message, but no callback is registered.");
      }) {
  ConfigureOptions(options);
}

CameraMessenger::~CameraMessenger() {
  // Make sure the camera is stopped.
  Stop();
}

void CameraMessenger::TranslateEncoded(void* buffer, size_t buffer_size,
                                       int64_t timestamp_us, uint32_t) {
  if (!on_message_ready_) {
    // Don't bother with the translation if we don't have a callback.
    return;
  }

  // Create the message for this image.
  sensor_msgs::Image message;

  FillHeader(&message.header, timestamp_us, image_message_sequence_++);

  message.height = stream_info_.height;
  message.width = stream_info_.width;
  message.step = stream_info_.stride;
  message.is_bigendian = false;
  message.encoding = ros_pixel_format_;

  // I don't get why people still use void pointers in the Year of Our Lord
  // 2022...
  const uint8_t* byte_buffer = static_cast<uint8_t*>(buffer);

  // Copy the raw image data
  const auto kExpectedStride = stream_info_.width * 3;
  if ((stream_info_.pixel_format == libcamera::formats::RGB888 ||
       stream_info_.pixel_format == libcamera::formats::BGR888) &&
      stream_info_.stride != kExpectedStride) {
    // If we have padding on the right edge of the buffer, we must copy it
    // without padding.
    message.data.resize(kExpectedStride * stream_info_.height);
    for (uint32_t i = 0; i < stream_info_.height; ++i) {
      std::copy_n(byte_buffer + i * stream_info_.stride, kExpectedStride,
                  message.data.data() + i * kExpectedStride);
    }

    message.step = kExpectedStride;
  } else {
    // If there is no padding, we can do it in a single copy.
    message.data.assign(byte_buffer, byte_buffer + buffer_size);
  }

  // WaitForFrame the callback with the new message.
  on_message_ready_(message);
}

void CameraMessenger::TranslateDetections(
    const CompletedRequestPtr& completed_request) {
  FrameDetections detections_message;

  FillHeaderFromMeta(&detections_message.header, completed_request,
                     detection_message_sequence_++);

  // Convert each detection.
  std::vector<postproc::Detection> detections;
  completed_request->post_process_metadata.Get("object_detect.results",
                                               detections);
  // It's normal for the "object_detect.results" tag to not be set if we don't
  // have any detections.

  bool is_rotated = false;
  completed_request->post_process_metadata.Get("object_detect.rotated",
                                               is_rotated);
  const auto kFrameWidth = static_cast<float>(
      !is_rotated ? stream_info_.width : stream_info_.height);
  const auto kFrameHeight = static_cast<float>(
      !is_rotated ? stream_info_.height : stream_info_.width);
  for (const auto& detection : detections) {
    Detection ros_detection;
    ros_detection.center_x = static_cast<float>(detection.box.x) / kFrameWidth;
    ros_detection.center_y = static_cast<float>(detection.box.y) / kFrameHeight;
    if (is_rotated) {
      ros_detection.center_x = 1.0 - ros_detection.center_x;
      ros_detection.center_y = 1.0 - ros_detection.center_y;
    }
    ros_detection.width = static_cast<float>(detection.box.width) / kFrameWidth;
    ros_detection.height =
        static_cast<float>(detection.box.height) / kFrameHeight;
    ros_detection.confidence = detection.confidence;
    ros_detection.class_id = detection.category;

    detections_message.detections.push_back(ros_detection);
  }

  // Copy the appearance features.
  std::vector<uint8_t> features;
  hailo_3d_image_shape_t features_shape{0, 0, 0};
  if (completed_request->post_process_metadata.Get("object_detect.features",
                                                   features) ||
      completed_request->post_process_metadata.Get(
          "object_detect.features_shape", features_shape)) {
    // Completed request had no appearance features, HAILO is probably not
    // active.
    ROS_DEBUG_STREAM(
        "A callback for detections was specified, but this frame has no "
        "appearance features.");
  }
  detections_message.appearance_features.resize(features.size());
  std::copy(features.begin(), features.end(),
            detections_message.appearance_features.begin());
  // Batch size is always one.
  detections_message.appearance_feature_shape[0] = 1;
  detections_message.appearance_feature_shape[1] = features_shape.height;
  detections_message.appearance_feature_shape[2] = features_shape.width;
  detections_message.appearance_feature_shape[3] = features_shape.features;

  // Call the callback with the new message.
  if (on_detections_ready_) {
    on_detections_ready_(detections_message);
  }
}

void CameraMessenger::TranslateMotion(
    const CompletedRequestPtr& completed_request) {
  uint32_t regions_with_motion;
  if (completed_request->post_process_metadata.Get(
          "motion_detect.regions_above_threshold", regions_with_motion)) {
    // This is a pretty common case, because usually we don't run motion
    // estimation on every frame.
    return;
  }

  // Fill in the message.
  FrameMotion motion_message;
  FillHeaderFromMeta(&motion_message.header, completed_request,
                     motion_message_sequence_++);
  motion_message.estimated_motion = regions_with_motion;

  // Call the callback with the new message.
  if (on_motion_ready_) {
    on_motion_ready_(motion_message);
  }
}

void CameraMessenger::SetMessageReadyCallback(
    const ImageReadyCallback& callback) {
  ROS_DEBUG_STREAM("Setting new callback for camera messages.");
  on_message_ready_ = callback;
}

void CameraMessenger::SetDetectionsReadyCallback(
    const DetectionsReadyCallback& callback) {
  ROS_DEBUG_STREAM("Setting new callback for detections.");
  on_detections_ready_ = callback;
}

void CameraMessenger::SetMotionReadyCallback(
    const MotionReadyCallback& callback) {
  ROS_DEBUG_STREAM("Setting new callback for motion.");
  on_motion_ready_ = callback;
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

  if (!camera_open_) {
    ROS_DEBUG_STREAM("Opening camera.");
    camera_app_->OpenCamera();
    camera_open_ = true;
  }
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
    // is bail out and have systemd restart the whole node.
    abort();
  } else if (message.type == RPiCamEncoder::MsgType::Quit) {
    ROS_INFO_STREAM("Got LibCamera quit request.");
    return false;
  }
  ROS_FATAL_STREAM_COND(message.type != RPiCamEncoder::MsgType::RequestComplete,
                        "Got unrecognized message type "
                            << static_cast<uint32_t>(message.type)
                            << " from LibCamera!");

  auto& completed_request = std::get<CompletedRequestPtr>(message.payload);
  camera_app_->EncodeBuffer(completed_request, camera_app_->VideoStream());
  TranslateDetections(completed_request);
  TranslateMotion(completed_request);

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
void CameraMessenger::FillHeader(std_msgs::Header* header,
                                 uint32_t timestamp_us,
                                 uint32_t sequence_num) const {
  // Sensor timestamps are from the kernel clock, but we want them relative to
  // the wall clock.
  const uint64_t kWallTimestampUs = KernelToRosClock(timestamp_us);
  header->stamp.sec = kWallTimestampUs / 1000000;
  header->stamp.nsec = (kWallTimestampUs % 1000000) * 1000;
  header->frame_id = frame_id_;
  header->seq = sequence_num;
}

void CameraMessenger::FillHeaderFromMeta(
    std_msgs::Header* header, const CompletedRequestPtr& completed_request,
    uint32_t sequence_num) const {
  // Sensor timestamps are from the kernel clock, but we want them relative to
  // the wall clock.
  if (const auto kSensorTimeNs =
          completed_request->metadata.get(controls::SensorTimestamp)) {
    FillHeader(header, *kSensorTimeNs / 1000, sequence_num);
  } else {
    // Just use current time.
    FillHeader(header, 0U, sequence_num);
    header->stamp = ros::Time::now();
  }
}

void CameraMessenger::ConfigureOptions(const VideoOptions& new_options) {
  // Copy the specified options to the camera app.
  auto* options = camera_app_->GetOptions();

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
  options->roi_x = 0;
  options->roi_y = 0;
  options->roi_width = 0;
  options->roi_height = 0;
  options->camera = new_options.camera;
  options->afMode_index = new_options.afMode_index;
  options->afWindow_x = new_options.afWindow_x;
  options->afWindow_y = new_options.afWindow_y;
  options->afWindow_width = new_options.afWindow_width;
  options->afWindow_height = new_options.afWindow_height;
  options->post_process_libs = new_options.post_process_libs;
  options->post_process_file = new_options.post_process_file;
  options->transform = new_options.transform;
  // Note: need to force the raw stream to off, otherwise it causes weird
  // resolution issues with the video output.
  options->no_raw = true;

  // Apply the new configuration.
  camera_app_->ReConfigureFromOptions();
}

void CameraMessenger::SetFocusLocked(bool locked) const {
  ROS_INFO_STREAM("Setting focus lock to " << locked);
  camera_app_->SetFocusLocked(locked);
}

void CameraMessenger::SetFrameDurationOffset(int32_t offset) const {
  ROS_DEBUG_STREAM("Setting frame duration offset to " << offset);
  camera_app_->SetFrameDurationOffset(offset);
}

}  // namespace libcamera_device
