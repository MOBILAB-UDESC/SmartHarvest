#ifndef SH_BASE_TEMPLATE__GRASP_GENERATOR_BASE_HPP_
#define SH_BASE_TEMPLATE__GRASP_GENERATOR_BASE_HPP_

#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/types/perception_types.hpp"

namespace sh_base_template
{

/**
 * @class sh_base_template::GraspGeneratorBase
 *
 * @brief Abstract interface for grasp generator plugins.
 *
 * A grasp generator computes grasp orientations for detected
 * objects based on perception results and scene information.
 */
class GraspGeneratorBase
{
public:
  explicit GraspGeneratorBase() = default;
  virtual ~GraspGeneratorBase() = default;

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
   * @brief Generate grasp orientations.
   *
   * Custom implementations must update the provided pose list with
   * grasp-related orientation
   *
   * @param frame Perception frame containing scene information.
   * @param poses List of poses to populate or update with grasp orientation.
   * @return true or false.
   */
  virtual bool generate_grasp(
    const sh_base_template::types::PerceptionFrame& frame,
    std::vector<sh_base_template::types::PoseFeatures>& poses) = 0;
};

}  // namespace sh_base_template

#endif  // SH_BASE_TEMPLATE__GRASP_GENERATOR_BASE_HPP_