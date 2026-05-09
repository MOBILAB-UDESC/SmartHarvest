#ifndef SH_BASE_TEMPLATE__LOCALISER_BASE_HPP_
#define SH_BASE_TEMPLATE__LOCALISER_BASE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_interfaces/msg/detection2_d.hpp"

namespace sh_base_template
{

/**
 * @class sh_base_template::LocaliserBase
 *
 * @brief Abstract interface for 2D object detector plugins.
 */
class LocaliserBase
{
public:
  explicit LocaliserBase() = default;
  virtual ~LocaliserBase() = default;

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
   * @brief Implementation of 3D pose estimation
   *
   * @param depth_input CV depth image.
   * @param cam_intrinsics Camera intrinsics matrix (array).
   * @param detections Detected objects list.
   * @param poses Poses list to populate.
   *
   * @return bool Localisation state.
   */
  virtual bool localise(
    const cv::Mat& depth_input,
    const std::string& depth_encoding,
    const std::array<double, 4UL>& cam_intrinsics,
    const std::vector<sh_interfaces::msg::Detection2D>& detections,
    std::vector<geometry_msgs::msg::Pose>& poses) = 0;

protected:

};

}  // namespace sh_base_template

#endif  // SH_BASE_TEMPLATE__LOCALISER_BASE_HPP_