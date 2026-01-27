/**
 * @file Custom YOLO inference stage designed for the MARS cameras.
 */

#ifndef MARS_CAMERA_MODULE_HAILO_MARS_YOLO_INFERENCE_HPP
#define MARS_CAMERA_MODULE_HAILO_MARS_YOLO_INFERENCE_HPP

#include <vector>

#include <libcamera/geometry.h>

#include "../object_detect.hpp"
#include "hailo_yolo_inference.hpp"

class MarsYoloInference : public YoloInference {
 public:
  ~MarsYoloInference() override = default;

 protected:
  std::vector<Detection> runInference(
      const uint8_t *frame,
      const std::vector<libcamera::Rectangle> &scaler_crops) override;

 private:
  /// Handle for the currently-running inference job.
  hailort::AsyncInferJob job_;
  /// Where to write output tensors from the job.
  std::vector<OutTensor> output_tensors_;
};

#endif  // MARS_CAMERA_MODULE_HAILO_MARS_YOLO_INFERENCE_HPP
