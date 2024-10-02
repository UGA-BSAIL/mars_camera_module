/**
 * @file Entry point for the node that reads camera data.
 */

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <libcamera_device/LibcameraDeviceConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

#include "camera_messenger.hpp"
#include "core/rpicam_encoder.hpp"
#include "core/video_options.hpp"

using dynamic_reconfigure::Server;
using image_transport::ImageTransport;
using image_transport::Publisher;
using libcamera_device::CameraMessenger;
using libcamera_device::LibcameraDeviceConfig;
using sensor_msgs::Image;

namespace {

// Unfortunately, Boost has its own placeholders that conflict with the std
// ones, so we have to rename them.
const auto kStd1 = std::placeholders::_1;
const auto kStd2 = std::placeholders::_2;

/**
 * @struct Represents static configuration, which is set once at startup.
 */
struct StaticConfig {
  /// The numeric identifier of the camera to read from.
  int32_t device_id;
};

/**
 * Publishes a camera message. This is meant to be used as a callback.
 * @param publisher Will be used for publishing images.
 * @param image The image to publish.
 */
void PublishEncoded(Publisher *publisher, const Image &image) {
  publisher->publish(image);
}

/**
 * @brief Translates the configuration struct used by `dynamic_reconfigure` to
 *  the one used by `libcamera`.
 * @param static_config The static parameter configuration.
 * @param dynamic_config The dynamic parameter configuration.
 * @param out_config [out] The camera configuration.
 */
void ParamToVideoConfig(const StaticConfig static_config,
                        const LibcameraDeviceConfig &dynamic_config,
                        VideoOptions *out_config) {
  out_config->framerate = static_cast<float>(dynamic_config.fps);
  out_config->mode =
      Mode(dynamic_config.width, dynamic_config.height, 24, false);
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

  out_config->ev = static_cast<float>(dynamic_config.ev);
  out_config->brightness = 0.0;
  out_config->contrast = static_cast<float>(dynamic_config.contrast);
  out_config->saturation = static_cast<float>(dynamic_config.saturation);
  out_config->sharpness = static_cast<float>(dynamic_config.sharpness);
}

/**
 * @brief Reconfigures the camera parameters. Meant to be used as a callback for
 *   dynamic reconfiguration.
 * @param messenger The `CameraMessenger` to reconfigure.
 * @param dynamic_config The configuration that was changed.
 * @param level The configuration level bitmask.
 */
void ReconfigureParams(CameraMessenger *messenger,
                       const StaticConfig &static_config,
                       const LibcameraDeviceConfig &dynamic_config,
                       uint32_t level) {
  ROS_INFO_STREAM("Reconfigure request: " << dynamic_config.width << "x"
                                          << dynamic_config.height << ", "
                                          << dynamic_config.fps << " FPS.");

  // Set the new parameters.
  VideoOptions camera_options;
  ParamToVideoConfig(static_config, dynamic_config, &camera_options);

  // Camera needs to be restarted for these to take effect.
  messenger->Stop();
  messenger->ConfigureOptions(camera_options);

  try {
      messenger->Start();
  } catch (const std::runtime_error &e) {
      ROS_FATAL_STREAM("Failed to start camera: " << e.what());
      // There's no easy way to recover from this. The best policy is to exit
      // and let systemd restart.
      exit(1);
  }
}

/**
 * @brief Ensures there is at least one subscriber to the camera topic(s) before
 *  continuing. If there is not one, it will stop the camera until there is.
 * @param publisher The camera publisher.
 * @param node The node handle.
 * @param camera The camera itself.
 */
void WaitForSubscriber(const Publisher &publisher, const ros::NodeHandle &node,
                       CameraMessenger *camera) {
  if (publisher.getNumSubscribers() > 0) {
    // We already have a subscriber, so we're done before we even started.
    return;
  }

  ros::Rate rate(5);

  // Wait for someone to subscribe. In the meantime, there's no point in
  // running the camera.
  camera->Stop();
  ROS_INFO_STREAM("Waiting for a camera subscriber...");
  while (node.ok() && publisher.getNumSubscribers() == 0) {
    rate.sleep();
    ros::spinOnce();
  }

  // Someone subscribed. Start the camera again.
  camera->Start();
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera", ros::init_options::AnonymousName);
  ros::NodeHandle node("~");

  // Read parameters.
  std::string camera_name, frame_id;
  int32_t device_id;
  node.param<std::string>("frame_id", frame_id, "frame");
  node.param<int32_t>("device_id", device_id, 0);

  ROS_INFO_STREAM("Starting camera node for device " << device_id << " and frame " << frame_id << "...");

  ImageTransport image_transport(node);
  auto image_publisher =
      image_transport.advertise(ros::this_node::getName(), 1);

  Server<LibcameraDeviceConfig> param_server;

  // Set the default configuration initially.
  const StaticConfig kStaticConfig = {device_id};
  LibcameraDeviceConfig default_dynamic_config;
  VideoOptions default_video_options;
  param_server.getConfigDefault(default_dynamic_config);
  ParamToVideoConfig(kStaticConfig, default_dynamic_config,
                     &default_video_options);

  // Set up the camera.
  CameraMessenger camera(std::make_unique<RPiCamEncoder>(), frame_id,
                         default_video_options);
  camera.SetMessageReadyCallback(
      std::bind(PublishEncoded, &image_publisher, kStd1));

  // Configure the dynamic reconfiguration callback.
  Server<LibcameraDeviceConfig>::CallbackType reconfigure_callback =
      std::bind(ReconfigureParams, &camera, kStaticConfig, kStd1, kStd2);
  param_server.setCallback(reconfigure_callback);

  // Process all camera messages.
  ros::Rate rate(5);
  // Wait for the camera to initialize.
  while (!camera.WaitForFrame()) {
    rate.sleep();
  }
  ROS_DEBUG_STREAM("Camera initialized!");
  while (node.ok() && camera.WaitForFrame()) {
    WaitForSubscriber(image_publisher, node, &camera);
    ros::spinOnce();
  }

  return 0;
}
