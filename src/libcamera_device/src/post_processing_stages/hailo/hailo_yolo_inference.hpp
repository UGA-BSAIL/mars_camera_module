#ifndef MARS_CAMERA_MODULE_HAILO_YOLO_INFERENCE_HPP
#define MARS_CAMERA_MODULE_HAILO_YOLO_INFERENCE_HPP

#include <libcamera/geometry.h>

#include <vector>

#include "../object_detect.hpp"
#include "detection/yolo_postprocess.hpp"
#include "hailo_postprocessing_stage.hpp"

class YoloInference : public HailoPostProcessingStage {
 public:
  explicit YoloInference(RPiCamApp *app);
  ~YoloInference() override;

  char const *Name() const override;

  void Read(boost::property_tree::ptree const &params) override;

  void Configure() override;

  bool Process(CompletedRequestPtr &completed_request) override;

 private:
  /**
   * @brief Runs an inference job on the HAILO accelerator.
   * @param frame The input frame to run inference on.
   * @param output_tensors The output tensors from the inference job.
   * @return True if the job succeeded, false otherwise.
   */
  bool runHailoJob(const uint8_t *frame,
                   std::vector<OutTensor> &output_tensors);

  virtual std::vector<postproc::Detection> runInference(
      const uint8_t *frame,
      const std::vector<libcamera::Rectangle> &scaler_crops,
      std::vector<OutTensor> &output_tensors);
  void filterOutputObjects(std::vector<postproc::Detection> &objects);

  struct LtObject {
    postproc::Detection params;
    unsigned int visible;
    unsigned int hidden;
    bool matched;
  };

  std::vector<LtObject> lt_objects_;
  std::mutex lock_;
  PostProcessingLib postproc_nms_;
  YoloParams *yolo_params_ = nullptr;

  // Config params
  std::string config_path_;
  std::string arch_;
  unsigned int max_detections_;
  float threshold_;
  bool temporal_filtering_;
  float tolerance_;
  float factor_;
  unsigned int visible_frames_;
  unsigned int hidden_frames_;
};

#endif  // MARS_CAMERA_MODULE_HAILO_YOLO_INFERENCE_HPP
