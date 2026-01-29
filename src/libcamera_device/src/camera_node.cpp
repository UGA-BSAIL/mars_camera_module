/**
 * @file Entry point for the node that reads camera data.
 */

#include <chrono>
#include <cstdlib>
#include <future>
#include <image_transport/image_transport.hpp>
#include <libcamera_device_msgs/msg/frame_detections.hpp>
#include <libcamera_device_msgs/msg/frame_motion.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include "camera_messenger.hpp"
#include "core/rpicam_encoder.hpp"
#include "core/video_options.hpp"

using image_transport::ImageTransport;
using libcamera_device::CameraMessenger;
using libcamera_device_msgs::msg::FrameDetections;
using libcamera_device_msgs::msg::FrameMotion;
using rcl_interfaces::msg::FloatingPointRange;
using rcl_interfaces::msg::IntegerRange;
using rcl_interfaces::msg::Parameter;
using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::ParameterEvent;
using sensor_msgs::msg::Image;

using ImagePublisher = image_transport::Publisher;
using DetectionPublisher = rclcpp::Publisher<FrameDetections>;
using MotionPublisher = rclcpp::Publisher<FrameMotion>;

namespace {

/// Names of parameters that require a full camera reset when they change.
const std::unordered_set<std::string> kParamRequiresReset = {
    "fps",      "width",      "height",    "ev",
    "contrast", "saturation", "sharpness", "postprocess_file"};

/**
 * Publishes a camera message. This is meant to be used as a callback.
 * @param publisher Will be used for publishing images.
 * @param image The image to publish.
 */
void PublishEncoded(const ImagePublisher &publisher, const Image &image) {
  publisher.publish(image);
}

/**
 * Publishes camera detections. This is meant to be used as a callback.
 * @param publisher Will be used for publishing detections.
 * @param detections The detections to publish.
 */
void PublishDetections(const DetectionPublisher::SharedPtr &publisher,
                       const FrameDetections &detections) {
  publisher->publish(detections);
}

/**
 * Publishes camera motion estimates. This is meant to be used as a callback.
 * @param publisher Will be used for publishing detections.
 * @param motion The detections to publish.
 */
void PublishMotion(const MotionPublisher::SharedPtr &publisher,
                   const FrameMotion &motion) {
  publisher->publish(motion);
}

/**
 * @brief Initializes the video options that never change.
 * @param out_config [out] The options structure to initialize.
 */
void InitConstantOptions(VideoOptions *out_config) {
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

  out_config->brightness = 0;
}

class CameraNode : public rclcpp::Node {
 public:
  explicit CameraNode(const rclcpp::NodeOptions &options)
      : Node("camera", options),
        param_subscriber_(
            std::make_shared<rclcpp::ParameterEventHandler>(this)) {
    // Parameter declarations
    // The FPS to request from the camera.
    ParameterDescriptor fps_descriptor;
    IntegerRange fps_range;
    fps_range.from_value = 1;
    fps_range.to_value = 120;
    fps_descriptor.integer_range.push_back(fps_range);
    declare_parameter("fps", 10, fps_descriptor);

    // The width of the image to request from the camera.
    ParameterDescriptor width_descriptor;
    IntegerRange width_range;
    width_range.from_value = 0;
    width_range.to_value = 10000;
    width_descriptor.integer_range.push_back(width_range);
    declare_parameter("width", 640, width_descriptor);

    // The height of the image to request from the camera.
    ParameterDescriptor height_descriptor;
    IntegerRange height_range;
    height_range.from_value = 0;
    height_range.to_value = 10000;
    height_descriptor.integer_range.push_back(height_range);
    declare_parameter("height", 480, height_descriptor);

    // Image exposure adjustment
    ParameterDescriptor ev_descriptor;
    FloatingPointRange ev_range;
    ev_range.from_value = -10.0;
    ev_range.to_value = 10.0;
    ev_descriptor.floating_point_range.push_back(ev_range);
    declare_parameter("ev", 0.0, ev_descriptor);

    // Image contrast adjustment
    ParameterDescriptor contrast_descriptor;
    FloatingPointRange contrast_range;
    contrast_range.from_value = 0.0;
    contrast_range.to_value = 10.0;
    contrast_descriptor.floating_point_range.push_back(contrast_range);
    declare_parameter("contrast", 1.0, contrast_descriptor);

    // Image saturation adjustment
    ParameterDescriptor saturation_descriptor;
    FloatingPointRange saturation_range;
    saturation_range.from_value = 0.0;
    saturation_range.to_value = 10.0;
    saturation_descriptor.floating_point_range.push_back(saturation_range);
    declare_parameter("saturation", 1.0, saturation_descriptor);

    // Image sharpness adjustment
    ParameterDescriptor sharpness_descriptor;
    FloatingPointRange sharpness_range;
    sharpness_range.from_value = 0.0;
    sharpness_range.to_value = 10.0;
    declare_parameter("sharpness", 1.0, sharpness_descriptor);

    // Specifies post-processing configuration to use.
    declare_parameter("postprocess_file", "");

    // Whether to lock the focus on cameras that support autofocus.
    declare_parameter("lock_focus", false);

    // Specifies which libcamera device to read from.
    declare_parameter("device_id", 0);

    // Specified the TF frame ID to use in output messages.
    declare_parameter("frame_id", "camera");

    // Initialize the video options using the parameters.
    InitVideoConfig();

    // Create a publisher for images.
    const auto node_shared = std::shared_ptr<Node>(this);
    image_transport_ = std::make_shared<ImageTransport>(node_shared);
    image_publisher_ = image_transport_->advertise(get_name(), 1);
    // Create a publisher for detections.
    detection_publisher_ =
        create_publisher<FrameDetections>("detections", 10);
    // Create a publisher for motion.
    motion_publisher_ = create_publisher<FrameMotion>("motion", 10);

    // Set up the publisher callbacks.
    camera_ = std::make_unique<CameraMessenger>(
        std::make_unique<RPiCamEncoder>(),
        get_parameter("frame_id").as_string(), options_, node_shared);
    camera_->SetMessageReadyCallback([this](const Image &image) {
      PublishEncoded(image_publisher_, image);
    });
    camera_->SetDetectionsReadyCallback(
        [this](const FrameDetections &detections) {
          PublishDetections(detection_publisher_, detections);
        });
    camera_->SetMotionReadyCallback([this](const FrameMotion &motion) {
      PublishMotion(motion_publisher_, motion);
    });

    // Set up the parameter callbacks.
    param_callback_ = param_subscriber_->add_parameter_event_callback(
        [this](const ParameterEvent &parameter_event) {
          ReconfigureParams(parameter_event);
        });
  }

  /**
   * @brief Ensures there is at least one subscriber to the camera topic(s)
   * before continuing. If there is not one, it will stop the camera until there
   * is.
   */
  void WaitForSubscriber() {
    if (!rclcpp::ok()) {
      return;
    }

    if (image_publisher_.getNumSubscribers() > 0 ||
        detection_publisher_->get_subscription_count() > 0 ||
        motion_publisher_->get_subscription_count() > 0) {
      // We already have a subscriber, so we're done before we even started.
      return;
    }

    // Wait for someone to subscribe. In the meantime, there's no point in
    // running the camera.
    camera_->Stop();
    RCLCPP_INFO(get_logger(), "Waiting for a camera subscriber...");
    const auto shared_node_ptr = shared_from_this();
    while (rclcpp::ok() && image_publisher_.getNumSubscribers() == 0 &&
           detection_publisher_->get_subscription_count() == 0 &&
           motion_publisher_->get_subscription_count() == 0) {
      rclcpp::spin_until_future_complete(shared_node_ptr,
                                         std::promise<bool>().get_future(),
                                         std::chrono::milliseconds(200));
    }

    if (rclcpp::ok()) {
      // Someone subscribed. Start the camera again.
      camera_->Start();
    }
  }

  /**
   * @brief Runs the camera until the node exits, handling new frames.
   */
  void Run() {
    RCLCPP_INFO_STREAM(get_logger(),
                       "Running camera node for device "
                           << get_parameter("device_id").as_int()
                           << " and frame "
                           << get_parameter("frame_id").as_string() << "...");

    // Process all camera messages.
    rclcpp::Rate rate(5);
    // Wait for the camera to initialize.
    camera_->Start();
    while (!camera_->WaitForFrame()) {
      rate.sleep();
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "Camera initialized!");

    const auto shared_node_ptr = shared_from_this();
    while (rclcpp::ok() && camera_->WaitForFrame()) {
      rclcpp::spin_some(shared_node_ptr);
      WaitForSubscriber();
    }
  }

 private:
  /// Configuration options passed to libcamera.
  VideoOptions options_;
  /// Manages the camera.
  std::unique_ptr<CameraMessenger> camera_;

  /// Image transport instance to use.
  std::shared_ptr<ImageTransport> image_transport_;
  /// Publishers for images, detections, and motion messages.
  ImagePublisher image_publisher_;
  DetectionPublisher::SharedPtr detection_publisher_;
  MotionPublisher::SharedPtr motion_publisher_;

  /// Subscriber for parameter events.
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  /// Handle for parameter callbacks.
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_callback_;

  /**
   * @brief Reconfigures the camera parameters. Meant to be used as a callback
   * for parameter events.
   * @param parameter_event A list of changed parameters.
   */
  void ReconfigureParams(const ParameterEvent &parameter_event) const {
    // Set the new parameters.
    VideoOptions new_options = options_;
    ParamsToVideoConfig(parameter_event.changed_parameters, &new_options);

    bool reset_camera = false;
    for (const auto &param_msg : parameter_event.changed_parameters) {
      const auto &param = rclcpp::Parameter::from_parameter_msg(param_msg);
      if (param.get_name() == "lock_focus") {
        // Set focus lock.
        RCLCPP_INFO_STREAM(get_logger(),
                           "Setting focus lock to " << param.as_bool());
        camera_->SetFocusLocked(param.as_bool());
      }

      if (kParamRequiresReset.contains(param.get_name())) {
        // Camera needs to be restarted for this parameter change to take
        // effect.
        reset_camera = true;
      }
    }

    if (reset_camera) {
      // Some parameters were changed which require a full reset.
      camera_->Stop();
      camera_->ConfigureOptions(new_options);

      try {
        camera_->Start();
      } catch (const std::runtime_error &e) {
        RCLCPP_FATAL_STREAM(get_logger(),
                            "Failed to start camera: " << e.what());
        // There's no easy way to recover from this. The best policy is to exit
        // and let systemd restart.
        exit(1);
      }
    }
  }

  /**
   * @brief Translates the configuration struct used by `dynamic_reconfigure` to
   *  the one used by `libcamera`.
   * @param parameters The changed parameters.
   * @param out_config [out] The camera configuration.
   */
  void ParamsToVideoConfig(const std::vector<Parameter> &parameters,
                           VideoOptions *out_config) const {
    InitConstantOptions(out_config);

    for (const auto &param_msg : parameters) {
      const auto &param = rclcpp::Parameter::from_parameter_msg(param_msg);
      if (param.get_name() == "fps") {
        RCLCPP_INFO_STREAM(get_logger(), "Updating FPS to " << param.as_int());
        out_config->framerate = param.as_int();
      } else if (param.get_name() == "width") {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Updating width to " << param.as_int());
        out_config->mode.width = param.as_int();
        out_config->width = param.as_int();
      } else if (param.get_name() == "height") {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Updating height to " << param.as_int());
        out_config->mode.height = param.as_int();
        out_config->height = param.as_int();
      } else if (param.get_name() == "device_id") {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Updating device ID to " << param.as_int());
        out_config->camera = param.as_int();
      } else if (param.get_name() == "ev") {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Updating EV to " << param.as_double());
        out_config->ev = param.as_double();
      } else if (param.get_name() == "contrast") {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Updating contrast to " << param.as_double());
        out_config->contrast = param.as_double();
      } else if (param.get_name() == "saturation") {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Updating saturation to " << param.as_double());
        out_config->saturation = param.as_double();
      } else if (param.get_name() == "sharpness") {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Updating sharpness to " << param.as_double());
        out_config->sharpness = param.as_double();
      } else if (param.get_name() == "postprocess_file") {
        RCLCPP_INFO_STREAM(
            get_logger(), "Updating postprocess_file to " << param.as_string());
        out_config->post_process_file = param.as_string();
      }
    }
  }

  /**
   * @brief Initializes the video options to sane values.
   */
  void InitVideoConfig() {
    InitConstantOptions(&options_);

    options_.framerate = get_parameter("fps").as_int();
    options_.mode = Mode(get_parameter("width").as_int(),
                         get_parameter("height").as_int(), 24, false);
    options_.width = get_parameter("width").as_int();
    options_.height = get_parameter("height").as_int();

    options_.camera = get_parameter("device_id").as_int();

    options_.ev = static_cast<float>(get_parameter("ev").as_double());
    options_.contrast =
        static_cast<float>(get_parameter("contrast").as_double());
    options_.saturation =
        static_cast<float>(get_parameter("saturation").as_double());
    options_.sharpness =
        static_cast<float>(get_parameter("sharpness").as_double());

    options_.post_process_file = get_parameter("postprocess_file").as_string();
  }
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  CameraNode node{rclcpp::NodeOptions()};
  node.Run();

  rclcpp::shutdown();

  return 0;
}
