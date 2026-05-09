#ifndef SH_BASE_TEMPLATE__GRASP_GENERATOR_BASE_HPP_
#define SH_BASE_TEMPLATE__GRASP_GENERATOR_BASE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace sh_base_template
{

/**
 * @class sh_base_template::GraspGeneratorBase
 *
 * @brief Abstract interface for 2D object detector plugins.
 */
class GraspGeneratorBase
{
public:
  explicit GraspGeneratorBase() = default;
  virtual ~GraspGeneratorBase() = default;

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
   * @brief Implementation of grasping orientation
   *
   * @param poses Poses list to populate with quaternion.
   *
   * @return bool Grasping Calculation state.
   */
  virtual bool generate_grasp(
    std::vector<geometry_msgs::msg::Pose>& poses) = 0;

protected:

};

}  // namespace sh_base_template

#endif  // SH_BASE_TEMPLATE__GRASP_GENERATOR_BASE_HPP_