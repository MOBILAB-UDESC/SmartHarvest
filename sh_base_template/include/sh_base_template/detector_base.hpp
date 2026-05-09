#ifndef SH_BASE_TEMPLATE__DETECTOR_BASE_HPP_
#define SH_BASE_TEMPLATE__DETECTOR_BASE_HPP_

#include "opencv2/opencv.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_interfaces/msg/detection2_d.hpp"

namespace sh_base_template
{

struct DetectorBaseConfig
{
  std::string model_path;
  std::vector<int64_t> input_size;
  bool use_gpu;
  bool inference_verbose;
};

/**
 * @class sh_base_template::DetectorBase
 *
 * @brief Abstract interface for 2D object detector plugins.
 */
class DetectorBase
{
public:
  explicit DetectorBase() = default;
  virtual ~DetectorBase() = default;

  /**
   * @brief Configures and initializes the detector.
   *
   * @param node Weak pointer to parent node.
   *
   * @return bool Configuration state.
   */
  virtual bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& /*node*/)
  {
    return true;
  }

  virtual void cleanup()
  {}

  /**
   * @brief Implementation of detection inference
   *
   * @param input CV input image.
   * @param detector_base_config Default params struct.
   * @param detections Output vector containing all detected objects.
   * The vector should be populated by the implementation.
   *
   * @return bool Detection state.
   */
  virtual bool detect(
    const cv::Mat& input,
    const DetectorBaseConfig& detector_base_config,
    std::vector<sh_interfaces::msg::Detection2D>& detections) = 0;

protected:
  /**
   * @brief Optional warmup routine.
   */
  virtual bool warmup(const cv::Mat& /*dummy_input*/)
  {
    return true;
  }

};

}  // namespace sh_base_template

#endif  // SH_BASE_TEMPLATE__DETECTOR_BASE_HPP_