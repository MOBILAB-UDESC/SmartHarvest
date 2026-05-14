#ifndef SH_RGBD_PERCEPTION_PIPELINE__RGBD_PERCEPTION_PIPELINE_HPP_
#define SH_RGBD_PERCEPTION_PIPELINE__RGBD_PERCEPTION_PIPELINE_HPP_

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sh_base_template/detector_base.hpp"
#include "sh_base_template/grasp_generator_base.hpp"
#include "sh_base_template/localiser_base.hpp"
#include "sh_base_template/perception_pipeline_base.hpp"
#include "sh_interfaces/msg/perception_input.hpp"
#include "sh_interfaces/msg/perception_scene.hpp"

namespace sh_rgbd_perception_pipeline
{

/**
 * @class sh_rgbd_perception_pipeline::RgbdPerceptionPipeline
 * @brief RGB-D perception pipeline plugin.
 *
 * This pipeline executes a perception workflow composed of:
 *   detector -> localiser -> grasp generator
 * The detector identifies objects in RGB-D data, the localiser estimates
 * their 3D poses, and the grasp generator computes grasp poses suitable
 * for robotic manipulation.
 */
class RgbdPerceptionPipeline : public sh_base_template::PerceptionPipelineBase
{
public:
  using ImageMsg = sensor_msgs::msg::Image;

  /**
   * @brief A constructor for sh_rgbd_perception_pipeline::RgbdPerceptionPipeline class.
   */
  explicit RgbdPerceptionPipeline();

  /**
   * @brief A destructor for sh_rgbd_perception_pipeline::RgbdPerceptionPipeline class.
   */
  ~RgbdPerceptionPipeline();

  /**
   * @brief Configure transition.
   *
   * Loads and configures the detector, localiser, and grasp generator plugins.
   * @param node Weak pointer to the parent lifecycle node.
   * @return true or false.
   */
  bool configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node) override;

  /**
   * @brief Activate transition.
   */
  void activate() override;

  /**
   * @brief Deactivate transition.
   */
  void deactivate() override;

  /**
   * @brief Cleanup transition.
   */
  void cleanup() override;

  /**
   * @brief Execute the RGB-D perception workflow.
   *
   * detector -> localiser -> grasp generator.
   * @param input Perception input containing RGB-D data.
   * @param output Perception scene populated with estimated poses and grasps.
   * @return true or false.
   */
  bool process(
    const sh_interfaces::msg::PerceptionInput& input,
    sh_interfaces::msg::PerceptionScene& output);

private:
  /**
   * @brief Reset plugin instances before unloading class loaders.
   */
  void reset_plugins()
  {
    detector_.reset();
    localiser_.reset();
    grasp_.reset();
  };

  pluginlib::ClassLoader<sh_base_template::DetectorBase> detector_loader_;
  pluginlib::ClassLoader<sh_base_template::LocaliserBase> localiser_loader_;
  pluginlib::ClassLoader<sh_base_template::GraspGeneratorBase> grasp_loader_;
  std::string detector_plugin_name_;
  std::string localiser_plugin_name_;
  std::string grasping_plugin_name_;
  pluginlib::UniquePtr<sh_base_template::DetectorBase> detector_;
  pluginlib::UniquePtr<sh_base_template::LocaliserBase> localiser_;
  pluginlib::UniquePtr<sh_base_template::GraspGeneratorBase> grasp_;
};

}  // namespace sh_rgbd_perception_pipeline


#endif  // SH_RGBD_PERCEPTION_PIPELINE__RGBD_PERCEPTION_PIPELINE_HPP_