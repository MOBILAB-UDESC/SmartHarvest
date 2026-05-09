#ifndef SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__MEAN_LOCALISER_HPP_
#define SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__MEAN_LOCALISER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sh_base_template/localiser_base.hpp"

namespace sh_default_plugins
{

/**
 * @class sh_default_plugins::LocaliserBase
 *
 * @brief Abstract interface for 2D object detector plugins.
 */
class MeanLocaliser: public sh_base_template::LocaliserBase
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
  bool localise(
    const cv::Mat& depth_input,
    const std::string& depth_encoding,
    const std::array<double, 4UL>& cam_intrinsics,
    const std::vector<sh_interfaces::msg::Detection2D>& detections,
    std::vector<geometry_msgs::msg::Pose>& poses);



protected:
  std::optional<std::array<double, 3>> compute_xyz(
    const cv::Mat& depth_input,
    const std::array<double, 4>& bbox,
    const std::array<double, 4UL>& cam_intrinsics);

  bool use_sim_time_;
  int points_;
  std::string depth_encoding_;
  double depth_scaling_;
};

}  // namespace sh_default_plugins

#endif  // SH_DEFAULT_PLUGINS__PERCEPTION_PLUGIN__MEAN_LOCALISER_HPP_