#ifndef SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__YOLO_DETECTOR_PY_HPP_
#define SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__YOLO_DETECTOR_PY_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sh_base_template/detector_base.hpp"
#include "sh_interfaces/srv/run_yolo_detection.hpp"

namespace sh_default_plugins
{

/**
 * @class sh_default_plugins::YoloDetectorPY.
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

  void cleanup() override;

  /**
   * @brief Process inputs and call a service for detection.
   *
   * @param input CV input image.
   * @param detector_base_config Default params struct.
   * @param detections Detection2D struct to populate.
   * @return bool Detection state.
   */
  bool detect(
    const cv::Mat& input,
    const sh_base_template::DetectorBaseConfig& detector_base_config,
    std::vector<sh_interfaces::msg::Detection2D>& detections);

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Service client members
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<sh_interfaces::srv::RunYoloDetection>::SharedPtr yolo_client_;
  std::shared_ptr<sh_interfaces::srv::RunYoloDetection::Request> yolo_request_;

  double min_confidence_threshold_;
  double iou_threshold_;
};

}  // namespace sh_default_plugins

#endif  // SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__YOLO_DETECTOR_PY_HPP_