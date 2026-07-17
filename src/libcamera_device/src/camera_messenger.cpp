#include "camera_messenger.hpp"

#include <hailort.h>
#include <libcamera/pixel_format.h>

#include <chrono>
#include <cstdlib>
#include <libcamera_device_msgs/msg/detection.hpp>
#include <limits>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <utility>
#include <vector>

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
};

// Timeout to use when waiting for a frame before we consider the camera
// stalled.
const std::chrono::seconds kCameraTimeout(1000);

using libcamera_device_msgs::msg::Detection;
using libcamera_device_msgs::msg::FrameDetections;
using libcamera_device_msgs::msg::FrameMotion;
using sensor_msgs::msg::Image;
using std_msgs::msg::Header;

}  // namespace

CameraMessenger::CameraMessenger(std::unique_ptr<RPiCamEncoder>&& camera_app,
                                 std::string frame_id,
                                 const VideoOptions& options,
                                 rclcpp::Node::SharedPtr node_handle)
    : node_(node_handle),
      camera_app_(std::move(camera_app)),
      frame_id_(std::move(frame_id)),
      on_message_ready_([=](const Image&) {
        // Default callback does nothing, but logs a warning.
        RCLCPP_INFO_STREAM(
            node_handle->get_logger(),
            "Got a camera message, but no callback is registered.");
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
  Image message;

  FillHeader(&message.header, timestamp_us);

  message.height = stream_info_.height;
  message.width = stream_info_.width;
  message.step = stream_info_.stride;
  message.is_bigendian = false;
  message.encoding = ros_pixel_format_;

  // I don't get why people still use void pointers in the Year of Our Lord
  // 2022...
  const uint8_t* byte_buffer = static_cast<uint8_t*>(buffer);
  // Copy raw image data.
  message.data.assign(byte_buffer, byte_buffer + buffer_size);

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

  const auto kFrameWidth = static_cast<float>(stream_info_.width);
  const auto kFrameHeight = static_cast<float>(stream_info_.height);
  for (const auto& detection : detections) {
    Detection ros_detection;
    ros_detection.center_x = static_cast<float>(detection.box.x) / kFrameWidth;
    ros_detection.center_y = static_cast<float>(detection.box.y) / kFrameHeight;
    ros_detection.width = static_cast<float>(detection.box.width) / kFrameWidth;
    ros_detection.height =
        static_cast<float>(detection.box.height) / kFrameHeight;
    ros_detection.confidence = detection.confidence;

    detections_message.detections.push_back(ros_detection);
  }

  // Copy the appearance features.
  hailo_3d_image_shape_t features_shape{0, 0, 0};
  {
    std::lock_guard<Metadata> lock(completed_request->post_process_metadata);
    const auto* features =
        completed_request->post_process_metadata
            .GetLocked<std::vector<uint8_t>>("object_detect.features");
    const auto* shape =
        completed_request->post_process_metadata
            .GetLocked<hailo_3d_image_shape_t>("object_detect.features_shape");
    if (features == nullptr || shape == nullptr) {
      // Completed request had no appearance features, HAILO is probably not
      // active.
      RCLCPP_DEBUG_STREAM(
          node_->get_logger(),
          "A callback for detections was specified, but this frame has no "
          "appearance features.");
    } else {
      detections_message.appearance_features.assign(features->begin(),
                                                    features->end());
      features_shape = *shape;
    }
  }
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
  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "Setting new callback for camera messages.");
  on_message_ready_ = callback;
}

void CameraMessenger::SetDetectionsReadyCallback(
    const DetectionsReadyCallback& callback) {
  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "Setting new callback for detections.");
  on_detections_ready_ = callback;
}

void CameraMessenger::SetMotionReadyCallback(
    const MotionReadyCallback& callback) {
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Setting new callback for motion.");
  on_motion_ready_ = callback;
}

void CameraMessenger::Start() {
  if (camera_running_) {
    // Already running.
    return;
  }
  camera_running_ = true;

  RCLCPP_INFO_STREAM(node_->get_logger(), "Starting camera.");

  // Set up the callback.
  camera_app_->SetEncodeOutputReadyCallback(std::bind(
      &CameraMessenger::TranslateEncoded, this, kStd1, kStd2, kStd3, kStd4));

  if (!camera_open_) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Opening camera.");
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

  RCLCPP_INFO_STREAM(node_->get_logger(), "Stopping camera.");
  camera_app_->StopCamera();
  camera_app_->StopEncoder();
  camera_app_->Teardown();
}

bool CameraMessenger::WaitForFrame() {
  if (!camera_running_) {
    return false;
  }

  RPiCamEncoder::Msg message = camera_app_->Wait(kCameraTimeout);
  if (!rclcpp::ok()) {
    // We got an exit request sometime during the wait.
    return false;
  }

  if (message.type == RPiCamEncoder::MsgType::Timeout) {
    RCLCPP_FATAL_STREAM(
        node_->get_logger(),
        "Timed out while waiting for a frame. This is either a hardware issue, "
        "or a bug in libcamera.");
    // It's not clear whether this is recoverable. Probably the best thing to do
    // is bail out and hove systemd restart the whole node.
    abort();
  } else if (message.type == RPiCamEncoder::MsgType::Quit) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Got LibCamera quit request.");
    return false;
  }
  RCLCPP_FATAL_STREAM_EXPRESSION(
      node_->get_logger(),
      message.type != RPiCamEncoder::MsgType::RequestComplete,
      "Got unrecognized message type " << static_cast<uint32_t>(message.type)
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
  RCLCPP_FATAL_STREAM_EXPRESSION(
      node_->get_logger(), encoding == kPixelFormatToEncoding.end(),
      "Got pixel format " << stream_info_.pixel_format
                          << ", which is not supported.");
  ros_pixel_format_ = encoding->second;
}
void CameraMessenger::FillHeader(Header* header, uint32_t timestamp_us) const {
  // Sensor timestamps are from the kernel clock, but we want them relative to
  // the wall clock.
  const uint64_t kWallTimestampUs = KernelToRosClock(timestamp_us);
  header->stamp.sec = kWallTimestampUs / 1000000;
  header->stamp.nanosec = (kWallTimestampUs % 1000000) * 1000;
  header->frame_id = frame_id_;
}

void CameraMessenger::FillHeaderFromMeta(
    Header* header, const CompletedRequestPtr& completed_request,
    uint32_t sequence_num) const {
  // Sensor timestamps are from the kernel clock, but we want them relative to
  // the wall clock.
  if (const auto kSensorTimeNs =
          completed_request->metadata.get(controls::SensorTimestamp)) {
    FillHeader(header, *kSensorTimeNs / 1000);
  } else {
    // Just use current time.
    FillHeader(header, 0U);
    header->stamp = node_->get_clock()->now();
  }
}

uint64_t CameraMessenger::KernelToRosClock(uint32_t timestamp_us) const {
  // Figure out the conversion.
  const auto kCurrentRosTime = node_->get_clock()->now();
  const auto kCurrentKernelTime = std::chrono::steady_clock::now();
  const auto kOffset = std::chrono::nanoseconds{kCurrentRosTime.nanoseconds()} -
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

void CameraMessenger::ConfigureOptions(const VideoOptions& new_options) const {
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
  options->camera = new_options.camera;
  options->afMode_index = new_options.afMode_index;
  options->afWindow_x = new_options.afWindow_x;
  options->afWindow_y = new_options.afWindow_y;
  options->afWindow_width = new_options.afWindow_width;
  options->afWindow_height = new_options.afWindow_height;
  options->post_process_libs = new_options.post_process_libs;
  options->post_process_file = new_options.post_process_file;
}

void CameraMessenger::SetFocusLocked(bool locked) const {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Setting focus lock to " << locked);
  camera_app_->SetFocusLocked(locked);
}

}  // namespace libcamera_device
