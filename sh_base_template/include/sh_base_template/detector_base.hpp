#ifndef SH_BASE_TEMPLATE__DETECTOR_BASE_HPP_
#define SH_BASE_TEMPLATE__DETECTOR_BASE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/types/perception_types.hpp"

namespace sh_base_template
{

/**
 * @class sh_base_template::DetectorBase
 * @brief Abstract interface for detector plugins.
 *
 * A detector performs object detection or segmentation using perception
 * input data such as RGB images, depth images, or point clouds.
 */
class DetectorBase
{
public:
  explicit DetectorBase() = default;
  virtual ~DetectorBase() = default;

  /**
   * @brief Configure transition.
   */
  virtual bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& /*node*/)
  {
    return true;
  }

  /**
   * @brief Activate transition.
   */
  virtual void activate() {};

  /**
   * @brief Deactivate transition.
   */
  virtual void deactivate() {};

  /**
   * @brief Cleanup transition.
   */
  virtual void cleanup() {};


  /**
   * @brief Executes model inference.
   *
   * Custom implementations must populate the detection output structure
   * with inference results.
   * @param detector_input Input data for the detector.
   * @param detector_output DetectorOutput to be populated with detection results.
   * @return true or false.
   */
  virtual bool detect(
    const sh_base_template::types::DetectorInput& detector_input,
    sh_base_template::types::DetectorOutput& detector_output) = 0;
};

}  // namespace sh_base_template

#endif  // SH_BASE_TEMPLATE__DETECTOR_BASE_HPP_