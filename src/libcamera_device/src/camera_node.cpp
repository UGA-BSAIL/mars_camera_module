/**
 * @file Entry point for the node that reads camera data.
 */

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <libcamera_device/FrameDetections.h>
#include <libcamera_device/LibcameraDeviceConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

#include "camera_messenger.hpp"
#include "core/rpicam_encoder.hpp"
#include "core/video_options.hpp"
#include "ros/console.h"

using dynamic_reconfigure::Server;
using image_transport::ImageTransport;
using libcamera_device::CameraMessenger;
using libcamera_device::LibcameraDeviceConfig;
using sensor_msgs::Image;
using ImagePublisher = image_transport::Publisher;

namespace {

const auto kStd1 = std::placeholders::_1;
const auto kStd2 = std::placeholders::_2;

struct StaticConfig {
  /// ID of the camera device.
  int32_t device_id;
  /// Whether to enable automatic mode selection for the camera.
  bool auto_mode_select;
};

/// Maps transform enum values to Libcamera transforms.
std::unordered_map<std::string, libcamera::Transform> kTransformMap{
    {"none", libcamera::Transform::Identity},
    {"h_flip", libcamera::Transform::HFlip},
};

/**
 * @brief Publishes the encoded image.
 * @param image_publisher The publisher for the image.
 * @param header_publisher A separate publisher for just the image header.
 * @param image The image message to publish.
 */
void PublishEncoded(const ImagePublisher* image_publisher,
                    const ros::Publisher* header_publisher,
                    const Image& image) {
  image_publisher->publish(image);
  header_publisher->publish(image.header);
}

void PublishDetections(const ros::Publisher* publisher,
                       const libcamera_device::FrameDetections& detections) {
  publisher->publish(detections);
}

/**
 * @brief Translates the configuration struct used by `dynamic_reconfigure` to
 *  the one used by `libcamera`.
 * @param static_config The static parameter configuration.
 * @param dynamic_config The dynamic parameter configuration.
 * @param out_config [out] The camera configuration.
 */
void ParamToVideoConfig(const StaticConfig static_config,
                        const LibcameraDeviceConfig& dynamic_config,
                        VideoOptions* out_config) {
  out_config->framerate = static_cast<float>(dynamic_config.fps);
  out_config->mode =
      Mode(dynamic_config.width, dynamic_config.height, 10, false);
  out_config->mode.framerate = dynamic_config.fps;
  out_config->width = dynamic_config.width;
  out_config->height = dynamic_config.height;

  out_config->camera = static_config.device_id;

  // Set options that are necessary, but that we don't support configuring
  // (yet). We don't have any use for preview mode.
  out_config->nopreview = true;
  // Don't use denoising.
  out_config->denoise = "off";
  // This just outputs the raw image data with no encoding.
  out_config->codec = "yuv420";

  // Autofocus is always enabled.
  out_config->afMode_index = libcamera::controls::AfModeContinuous;
  // Set it to use the center of the frame for autofocus to avoid confusion from
  // MARS structural elements.
  out_config->afWindow_x = 0.25;
  out_config->afWindow_y = 0.25;
  out_config->afWindow_width = 0.5;
  out_config->afWindow_height = 0.5;

  out_config->ev = static_cast<float>(dynamic_config.ev);
  out_config->brightness = 0.0;
  out_config->contrast = static_cast<float>(dynamic_config.contrast);
  out_config->saturation = static_cast<float>(dynamic_config.saturation);
  out_config->sharpness = static_cast<float>(dynamic_config.sharpness);

  out_config->post_process_file = dynamic_config.postprocess_file;

  // Disabling the raw stream will force it to not auto-select the sensor mode.
  out_config->no_raw = !static_config.auto_mode_select;
  ROS_WARN_STREAM_COND(!static_config.auto_mode_select,
                       "Disabling automatic mode selection for sensor!");

  // Configure transformation.
  out_config->transform = libcamera::Transform::Identity;
  if (const auto kTransform = kTransformMap.find(dynamic_config.transform);
      kTransform == kTransformMap.end()) {
    ROS_ERROR_STREAM("Got unknown transform type '" << dynamic_config.transform
                                                    << "', ignoring.");
  } else {
    out_config->transform = kTransform->second;
  }
}

/**
 * @brief Reconfigures the camera parameters. Meant to be used as a callback for
 *   dynamic reconfiguration.
 * @param messenger The `CameraMessenger` to reconfigure.
 * @param dynamic_config The configuration that was changed.
 * @param level The configuration level bitmask.
 */
void ReconfigureParams(CameraMessenger* messenger,
                       const StaticConfig& static_config,
                       const LibcameraDeviceConfig& dynamic_config,
                       uint32_t level) {
  // Set the new parameters.
  VideoOptions camera_options;
  ParamToVideoConfig(static_config, dynamic_config, &camera_options);

  if (level & 0x8) {
    // Restart the camera.
    ROS_INFO_STREAM("Restarting camera due to configuration change...");
    messenger->Stop();
  }
  if (level & 0x1) {
    // Set standard options.
    ROS_INFO_STREAM("Reconfigure request: " << dynamic_config.width << "x"
                                            << dynamic_config.height << ", "
                                            << dynamic_config.fps << " FPS.");
    messenger->ConfigureOptions(camera_options);
  }
  if (level & 0x2) {
    // Set frame duration offset.
    messenger->SetFrameDurationOffset(dynamic_config.frame_duration_offset);
  }
  if (level & 0x4) {
    // Set focus lock.
    messenger->SetFocusLocked(dynamic_config.lock_focus);
  }
}

/**
 * @brief Ensures there is at least one subscriber to the camera topic(s) before
 *  continuing. If there is not one, it will stop the camera until there is.
 * @param image_publisher The camera image publisher.
 * @param detection_publisher The camera detection publisher.
 * @param node The node handle.
 * @param camera The camera itself.
 */
void WaitForSubscriber(const ImagePublisher& image_publisher,
                       const ros::Publisher& detection_publisher,
                       const ros::NodeHandle& node, CameraMessenger* camera) {
  if (image_publisher.getNumSubscribers() > 0 ||
      detection_publisher.getNumSubscribers() > 0) {
    // We already have a subscriber, so we're done before we even started.
    return;
  }

  ros::Rate rate(5);

  // Wait for someone to subscribe. In the meantime, there's no point in
  // running the camera.
  camera->Stop();
  ROS_INFO_STREAM("Waiting for a camera subscriber...");
  while (node.ok() && image_publisher.getNumSubscribers() == 0 &&
         detection_publisher.getNumSubscribers() == 0) {
    rate.sleep();
    ros::spinOnce();
  }

  // Someone subscribed. Start the camera again.
  camera->Start();
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera", ros::init_options::AnonymousName);
  ros::NodeHandle node("~");

  // Read parameters.
  std::string camera_name, frame_id;
  int32_t device_id;
  bool mode_select;
  node.param<std::string>("frame_id", frame_id, "frame");
  node.param<int32_t>("device_id", device_id, 0);
  node.param<bool>("enable_mode_select", mode_select, true);

  ROS_INFO_STREAM("Starting camera node for device "
                  << device_id << " and frame " << frame_id << "...");

  // Create a publisher for images.
  ImageTransport image_transport(node);
  auto image_publisher =
      image_transport.advertise(ros::this_node::getName(), 1);
  // Create a publisher for detections.
  auto detection_publisher =
      node.advertise<libcamera_device::FrameDetections>("detections", 10);
  // Create a publisher for just the frame headers.
  ros::Publisher header_publisher =
      node.advertise<std_msgs::Header>("frame_headers", 10);

  Server<LibcameraDeviceConfig> param_server;

  // Set the default configuration initially.
  const StaticConfig kStaticConfig = {device_id, mode_select};
  LibcameraDeviceConfig default_dynamic_config;
  VideoOptions default_video_options;
  param_server.getConfigDefault(default_dynamic_config);
  ParamToVideoConfig(kStaticConfig, default_dynamic_config,
                     &default_video_options);

  // Set up the camera.
  CameraMessenger camera(std::make_unique<RPiCamEncoder>(), frame_id,
                         default_video_options);
  camera.SetMessageReadyCallback(
      std::bind(PublishEncoded, &image_publisher, &header_publisher, kStd1));
  camera.SetDetectionsReadyCallback(
      std::bind(PublishDetections, &detection_publisher, kStd1));

  // Configure the dynamic reconfiguration callback.
  Server<LibcameraDeviceConfig>::CallbackType reconfigure_callback =
      std::bind(ReconfigureParams, &camera, kStaticConfig, kStd1, kStd2);
  param_server.setCallback(reconfigure_callback);

  // Process all camera messages.
  camera.Start();
  ros::Rate rate(5);
  // Wait for the camera to initialize.
  while (!camera.WaitForFrame()) {
    rate.sleep();
  }
  ROS_DEBUG_STREAM("Camera initialized!");
  while (node.ok() && camera.WaitForFrame()) {
    WaitForSubscriber(image_publisher, detection_publisher, node, &camera);
    ros::spinOnce();
  }

  return 0;
}
