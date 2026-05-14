#ifndef SH_RGBD_PERCEPTION_PIPELINE__YOLO_DETECTOR_PY_HPP_
#define SH_RGBD_PERCEPTION_PIPELINE__YOLO_DETECTOR_PY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/detector_base.hpp"
#include "sh_interfaces/srv/run_yolo_detection.hpp"

namespace sh_rgbd_perception_pipeline
{

struct DetectorConfig
{
  bool inference_verbose;
  bool use_gpu;
  double iou_threshold;
  double min_confidence_threshold;
  // std::string model_path;
  std::vector<int64_t> input_size;
};

/**
 * @class sh_rgbd_perception_pipeline::YoloDetectorPY.
 * @brief C++ wrapper that forwards detection requests to a Python-based YOLO inference server.
 */
class YoloDetectorPY : public sh_base_template::DetectorBase
{
public:
  /**
   * @brief Configure custom parameters and creates a service client.
   *
   * @param node Weak pointer to parent node.
   * @return bool.
   */
  bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& node) override;

  /**
   * @brief Cleanup transition.
   */
  void cleanup() override;

  /**
   * @brief Process inputs and call a service for detection.
   *
   * @param detector_input Input data for the detector (RGB image).
   * @param detector_output Output populated with detection results.
   * @return true or false.
   */
  bool detect(
    const sh_base_template::types::DetectorInput& detector_input,
    sh_base_template::types::DetectorOutput& detector_output);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Service client members
  rclcpp::Client<sh_interfaces::srv::RunYoloDetection>::SharedPtr yolo_client_;
  std::shared_ptr<sh_interfaces::srv::RunYoloDetection::Request> yolo_request_;

  DetectorConfig detector_config_;
};

}  // namespace sh_rgbd_perception_pipeline

#endif  // SH_RGBD_PERCEPTION_PIPELINE__YOLO_DETECTOR_PY_HPP_