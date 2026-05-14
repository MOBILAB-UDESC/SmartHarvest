#ifndef SH_RGBD_PERCEPTION_PIPELINE__MEAN_LOCALISER_HPP_
#define SH_RGBD_PERCEPTION_PIPELINE__MEAN_LOCALISER_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/localiser_base.hpp"

namespace sh_rgbd_perception_pipeline
{

struct CameraIntrinsics
{
  double f_x;
  double f_y;
  double c_x;
  double c_y;
};

/**
 * @class sh_rgbd_perception_pipeline::MeanLocaliser.
 * @brief Localisation plugin that estimates 3D object positions from depth data.
 */
class MeanLocaliser: public sh_base_template::LocaliserBase
{
public:
  /**
   * @brief Configure custom parameters.
   *
   * @param node Weak pointer to parent node.
   * @return bool.
   */
  bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& node) override;

  /**
   * @brief Cleanup transition.
   */
  void cleanup() override;

  /**
   * @brief 3D pose estimation.
   *
   * @param localiser_input Input data for localisation.
   * @param localiser_output Output populated with valid estimated object poses.
   * @return bool Localisation state.
   */
  bool localise(
    const sh_base_template::types::LocaliserInput& localiser_input,
    sh_base_template::types::LocaliserOutput& localiser_output);

protected:
  /**
   * @brief Computes object positions using depth values sampled inside each bbox.
   *
   * @param depth_input Depth image.
   * @param bbox Bounding box coordinates and sizes.
   * @return Estimated XYZ position, or std::nullopt if estimation fails.
   */
  std::optional<std::array<double, 3>> compute_xyz(
    const cv::Mat& depth_input,
    const std::array<double, 4>& bbox);

  int points_;
  std::vector<double> camera_to_end_effector_transform_;

  std::string depth_encoding_;
  double depth_scaling_;

  CameraIntrinsics camera_intrinsics_;
};

}  // namespace sh_rgbd_perception_pipeline

#endif  // SH_RGBD_PERCEPTION_PIPELINE__MEAN_LOCALISER_HPP_