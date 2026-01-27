#ifndef ROS_LIBCAMERA_CAMERA_MESSENGER_HPP
#define ROS_LIBCAMERA_CAMERA_MESSENGER_HPP

#include <functional>
#include <libcamera_device_msgs/msg/frame_detections.hpp>
#include <libcamera_device_msgs/msg/frame_motion.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
  using ImageReadyCallback =
      std::function<void(const sensor_msgs::msg::Image &)>;
  /**
   * @brief Type of the callback that will be fired whenever a new detection
   *    message is produced. It takes a single argument, which is the message.
   */
  using DetectionsReadyCallback =
      std::function<void(const libcamera_device_msgs::msg::FrameDetections &)>;
  /**
   * @brief Type of the callback that will be fired whenever a new motion
   *    message is produced. It takes a single argument, which is the message.
   */
  using MotionReadyCallback =
      std::function<void(const libcamera_device_msgs::msg::FrameMotion &)>;

  /**
   * @param camera_app The camera app to read data from.
   * @param frame_id The frame ID to use for published messages.
   * @param node_handle The ROS2 node handle.
   */
  explicit CameraMessenger(std::unique_ptr<RPiCamEncoder> &&camera_app,
                           std::string frame_id, const VideoOptions &options,
                           rclcpp::Node::SharedPtr node_handle);

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
  void SetMessageReadyCallback(const ImageReadyCallback &callback);
  /**
   * Sets the callback that will be invoked whenever new detections are ready.
   * @param callback The callback to set.
   */
  void SetDetectionsReadyCallback(const DetectionsReadyCallback &callback);
  /**
   * Sets the callback that will be invoked whenever new motion estimates are
   *    ready.
   * @param callback The callback to set.
   */
  void SetMotionReadyCallback(const MotionReadyCallback &callback);

  /**
   * @brief Sets new options for the camera.
   */
  void ConfigureOptions(const VideoOptions &new_options) const;

  /**
   * @brief Sets whether auto-focus is locked
   * @param locked True iff auto-focus should be locked.
   */
  void SetFocusLocked(bool locked) const;

 private:
  /// True iff camera is currently running.
  bool camera_running_ = false;
  /// True iff camera is currently open.
  bool camera_open_ = false;
  /// ROS node handle.
  rclcpp::Node::SharedPtr node_;
  /// Camera app that we will read frames from.
  std::unique_ptr<RPiCamEncoder> camera_app_;
  /// Information about the video stream from the camera.
  StreamInfo stream_info_;
  /// Associated ROS pixel format.
  std::string ros_pixel_format_;
  /// The frame ID to use for sent messages.
  std::string frame_id_;

  /// Callback to invoke when a new image message is ready.
  ImageReadyCallback on_message_ready_;
  /// Callback to invoke when new detections are ready.
  DetectionsReadyCallback on_detections_ready_;
  /// Callback to invoke when a new motion estimate is ready.
  MotionReadyCallback on_motion_ready_;

  /// Maintains the sequence number for messages we produce.
  uint32_t image_message_sequence_ = 0;
  uint32_t detection_message_sequence_ = 0;
  uint32_t motion_message_sequence_ = 0;

  /**
   * Callback for the encoder that translates an image into a ROS message.
   * Will call the proper callback internally.
   * @param buffer The buffer containing the image data.
   * @param buffer_size The size of the buffer in bytes.
   * @param timestamp_us The associated timestamp, in microseconds.
   * @param flags Associated flags from the encoder.
   */
  void TranslateEncoded(void *buffer, size_t buffer_size, int64_t timestamp_us,
                        uint32_t flags);
  /**
   * Callback for the encoder that translates a detections message into a ROS
   * message. Will call the proper callback internally.
   * @param completed_request The camera message.
   */
  void TranslateDetections(const CompletedRequestPtr &completed_request);

  /**
   * Callback for the encoder that translates a motion message into a ROS
   * message. Will call the proper callback internally.
   * @param completed_request The camera message.
   */
  void TranslateMotion(const CompletedRequestPtr &completed_request);

  /**
   * @brief Updates the stream information, based on the currently-configured
   *    camera.
   */
  void UpdateStreamInfo();

  /**
   * @brief Fills in the header for a message from the camera.
   * @param header The header to fill.
   * @param timestamp_us The sensor timestamp in microseconds.
   */
  void FillHeader(std_msgs::msg::Header *header, uint32_t timestamp_us) const;

  /**
   * @brief Fills in the header for a message from the camera.
   * @param header The header to fill.
   * @param completed_request The request structure containing the metadata.
   * @param sequence_num The sequence number to use.
   */
  void FillHeaderFromMeta(std_msgs::msg::Header *header,
                          const CompletedRequestPtr &completed_request,
                          uint32_t sequence_num) const;

  /**
   * @brief Converts a kernel timestamp to ROS time.
   * @param timestamp_us The kernel timestamp, in uS.
   * @return The ROS timestamp, in uS.
   */
  uint64_t KernelToRosClock(uint32_t timestamp_us) const;
};

}  // namespace libcamera_device

#endif  // ROS_LIBCAMERA_CAMERA_MESSENGER_HPP
