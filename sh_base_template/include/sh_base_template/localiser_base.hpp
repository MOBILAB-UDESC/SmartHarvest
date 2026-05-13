#ifndef SH_BASE_TEMPLATE__LOCALISER_BASE_HPP_
#define SH_BASE_TEMPLATE__LOCALISER_BASE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/types/perception_types.hpp"

namespace sh_base_template
{

/**
 * @class sh_base_template::LocaliserBase
 *
 * @brief Abstract interface for localiser plugins.
 *
 * A localiser estimates 3D object position from perception data and outputs.
 */
class LocaliserBase
{
public:
  explicit LocaliserBase() = default;
  virtual ~LocaliserBase() = default;

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
   * @brief 3D pose estimation.
   *
   * Custom implementations must populate the localisation output with
   * valid estimated poses
   * @param localiser_input Input data for localisation.
   * @param localiser_output Output populated with valid estimated object poses.
   * @return bool Localisation state.
   */
  virtual bool localise(
    const sh_base_template::types::LocaliserInput& localiser_input,
    sh_base_template::types::LocaliserOutput& localiser_output) = 0;
};

}  // namespace sh_base_template

#endif  // SH_BASE_TEMPLATE__LOCALISER_BASE_HPP_