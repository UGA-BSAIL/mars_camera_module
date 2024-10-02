#ifndef ROS_LIBCAMERA_CAMERA_MESSENGER_HPP
#define ROS_LIBCAMERA_CAMERA_MESSENGER_HPP

#include <sensor_msgs/Image.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "core/rpicam_encoder.hpp"
#include "core/video_options.hpp"

namespace libcamera_device {

/**
 * @class Translates camera data coming from a `LibcameraApp` to messages
 * that can be published using `ImageTransport`.
 */
class CameraMessenger {
 public:
  /**
   * @brief Type of the callback that will be fired whenever a new message is
   *    produced. It takes a single argument, which is the message.
   */
  using MessageReadyCallback = std::function<void(const sensor_msgs::Image &)>;

  /**
   * @param camera_app The camera app to read data from.
   * @param frame_id The frame ID to use for published messages.
   */
  explicit CameraMessenger(std::unique_ptr<RPiCamEncoder> &&camera_app,
                           std::string frame_id, const VideoOptions &options);

  ~CameraMessenger();

  /**
   * @brief Starts the camera. Must be called before `WaitForFrame()`.
   */
  void Start();

  /**
   * @brief Reads a single frame from the camera.
   * @return True if it successfully got a frame, false otherwise.
   */
  bool WaitForFrame();

  /** @brief Stops the camera. */
  void Stop();

  /**
   * Sets the callback that will be invoked whenever a new message is ready.
   * @param callback The callback to set.
   */
  void SetMessageReadyCallback(const MessageReadyCallback &callback);

  /**
   * @brief Sets new options for the camera.
   */
  void ConfigureOptions(const VideoOptions &new_options);

 private:
  /// True iff camera is currently running.
  bool camera_running_ = false;
  /// Camera app that we will read frames from.
  std::unique_ptr<RPiCamEncoder> camera_app_;
  /// Information about the video stream from the camera.
  StreamInfo stream_info_;
  /// Associated ROS pixel format.
  std::string ros_pixel_format_;
  /// The frame ID to use for sent messages.
  std::string frame_id_;

  /// Callback to invoke when a new message is ready.
  MessageReadyCallback on_message_ready_;

  /// Maintains the sequence number for messages we produce.
  uint32_t message_sequence_ = 0;

  /**
   * Callback for the encoder that translates an image into a ROS message.
   * @param buffer The buffer containing the image data.
   * @param buffer_size The size of the buffer in bytes.
   * @param timestamp_us The associated timestamp, in microseconds.
   * @param flags Associated flags from the encoder.
   */
  void TranslateEncoded(void *buffer, size_t buffer_size, int64_t timestamp_us,
                        uint32_t flags);

  /**
   * @brief Updates the stream information, based on the currently-configured
   *    camera.
   */
  void UpdateStreamInfo();
};

}  // namespace libcamera_device

#endif  // ROS_LIBCAMERA_CAMERA_MESSENGER_HPP
