#include "hailo_mars_yolo_inference.hpp"

std::vector<Detection> MarsYoloInference::runInference(
    const uint8_t *frame,
    const std::vector<libcamera::Rectangle> &scaler_crops) {
  std::vector<OutTensor> output_tensors;
  if (!runHailoJob(frame, output_tensors)) {
    return {};
  }

  return {};
}
