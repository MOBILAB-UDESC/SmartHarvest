#ifndef SH_RGBD_PERCEPTION_PIPELINE__DEFAULT_GRASP_GENERATOR_HPP_
#define SH_RGBD_PERCEPTION_PIPELINE__DEFAULT_GRASP_GENERATOR_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/grasp_generator_base.hpp"
#include "sh_base_template/types/perception_types.hpp"

namespace sh_rgbd_perception_pipeline
{

/**
 * @class sh_rgbd_perception_pipeline::BasicGraspGenerator.
 * @brief Grasp generator plugin that computes grasp orientations based on
 * object position relative to the camera frame.
 *
 * This implementation computes rotations that align the end-effector
 * toward the detected object centre.
 */
class DefaultGraspGenerator: public sh_base_template::GraspGeneratorBase
{
public:
  /**
   * @brief Configures transition.
   *
   * @param node Weak pointer to parent node.
   *
   * @return bool Configuration state.
   */
  bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& node) override;

  /**
   * @brief Cleanup transition
   */
  void cleanup() override;

  /**
   * @brief Generate grasp orientations from object positions.
   *
   * Computes rotational offsets to align the end-effector with the detected
   * object centre in the camera frame.
   * @param frame Perception frame containing scene information.
   * @param poses List of poses to populate or update with grasp orientation.
   * @return true or false.
   */
  bool generate_grasp(
    const sh_base_template::types::PerceptionFrame& /*frame*/,
    std::vector<sh_base_template::types::PoseFeatures>& poses);

protected:
  bool use_sim_time_;
  std::vector<double> camera_to_end_effector_transform_;
};

}  // namespace sh_rgbd_perception_pipeline

#endif  // SH_RGBD_PERCEPTION_PIPELINE__DEFAULT_GRASP_GENERATOR_HPP_