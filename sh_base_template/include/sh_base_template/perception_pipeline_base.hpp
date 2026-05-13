#ifndef SH_BASE_TEMPLATE__PERCEPTION_PIPELINE_BASE_HPP_
#define SH_BASE_TEMPLATE__PERCEPTION_PIPELINE_BASE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_interfaces/msg/perception_input.hpp"
#include "sh_interfaces/msg/perception_scene.hpp"

namespace sh_base_template
{

/**
 * @class sh_base_template::PerceptionPipelineBase
 * @brief Abstract interface for perception pipeline plugins.
 *
 * A perception pipeline defines the complete flow of a perception task,
 * including object detection, localisation, and optional post-processing
 * such as grasp generation.
 */
class PerceptionPipelineBase
{
public:
  /**
   * @brief Configure transition.
   */
  virtual bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
  {
    return true;
  };

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
   * @brief Executes the perception pipeline.
   *
   * Custom implementations must define the perception workflow and populate
   * the output scene with estimated object poses and related information.
   * @param input PerceptionInput.
   * @param output PerceptionScene to be populated with perception results.
   * @return true or false.
   */
  virtual bool process(
    const sh_interfaces::msg::PerceptionInput& input,
    sh_interfaces::msg::PerceptionScene& output) = 0;
};

} // namespace sh_base_template


#endif  // SH_BASE_TEMPLATE__PERCEPTION_PIPELINE_BASE_HPP_