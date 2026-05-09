#ifndef SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__DEFAULT_GRASP_GENERATOR_HPP_
#define SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__DEFAULT_GRASP_GENERATOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sh_base_template/grasp_generator_base.hpp"

namespace sh_default_plugins
{

/**
 * @class sh_default_plugins::GraspGeneratorBase
 *
 * @brief Abstract interface for 2D object detector plugins.
 */
class DefaultGraspGenerator: public sh_base_template::GraspGeneratorBase
{
public:
  /**
   * @brief Configures and initializes the detector.
   *
   * @param node Weak pointer to parent node.
   *
   * @return bool Configuration state.
   */
  bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& node) override;

  void cleanup() override;

  /**
   * @brief Implementation of 3D pose estimation
   *
   * @param depth_input CV depth image.
   * @param cam_intrinsics Camera intrinsics matrix (array).
   * @param detections Detected objects list.
   * @param poses Poses list to populate.
   *
   * @return bool Localisation state.
   */
  bool generate_grasp(
    std::vector<geometry_msgs::msg::Pose>& poses);



protected:
  bool use_sim_time_;
  std::vector<double> camera_to_end_effector_transform_;
};

}  // namespace sh_default_plugins

#endif  // SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__DEFAULT_GRASP_GENERATOR_HPP_