#ifndef SH_PERCEPTION_SERVER__PERCEPTION_SERVER_HPP_
#define SH_PERCEPTION_SERVER__PERCEPTION_SERVER_HPP_

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/detector_base.hpp"
#include "sh_base_template/grasp_generator_base.hpp"
#include "sh_base_template/localiser_base.hpp"
#include "sh_interfaces/srv/run_perception.hpp"

namespace sh_perception_server
{

using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using CallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;
using ImageMsg = sensor_msgs::msg::Image;

class PerceptionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief A constructor for sh_perception_server::PerceptionServer class.
   *
   * @param node_name Name of the node.
   */
  explicit PerceptionServer(const std::string & node_name);

  /**
   * @brief A destructor for sh_perception_server::PerceptionServer class.
   */
  ~PerceptionServer() = default;

protected:
  /**
   * @brief Callback function for configure transition.
   *
   * Loads detector, localiser, and grasp plugins.
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback function for activate transition.
   *
   * Creates a service for object detection and 3D pose estimation.
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for deactivate transition.

  * @param state A reference to the state of the Lifecycle Node.
  * @return SUCCESS or FAILURE.
  */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for cleanup transition.

  * @param state A reference to the state of the Lifecycle Node.
  * @return SUCCESS or FAILURE.
  */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for shutdown transition.

  * @param state A reference to the state of the Lifecycle Node.
  * @return SUCCESS or FAILURE.
  */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Handle detection service requests.
   *
   * @param request Service request containing RGB-D images and camera info.
   * @param response Service response to populate with detection results.
   */
  void trigger_detections_callback(
    const std::shared_ptr<sh_interfaces::srv::RunPerception::Request> request,
    std::shared_ptr<sh_interfaces::srv::RunPerception::Response> response);

  /**
   * @brief Declare parameters.
   */
  void declare_parameters();

  /**
   * @brief Annotate detected objects info onto a CV image.
   *
   * @param output CV image to publish.
   * @param detections Detected objects info.
   */
  void annotate_ouput(
    cv::Mat& output,
    const std::vector<sh_interfaces::msg::Detection2D>& detections);

  /**
   * @brief Convert cv image to ImageMsg and publishes it.
   *
   * @param output CV image to publish.
   */
  void publish_output(const cv::Mat& output);

  pluginlib::ClassLoader<sh_base_template::DetectorBase> detector_loader_;
  pluginlib::ClassLoader<sh_base_template::LocaliserBase> localiser_loader_;
  pluginlib::ClassLoader<sh_base_template::GraspGeneratorBase> grasp_loader_;
  pluginlib::UniquePtr<sh_base_template::DetectorBase> detector_;
  pluginlib::UniquePtr<sh_base_template::LocaliserBase> localiser_;
  pluginlib::UniquePtr<sh_base_template::GraspGeneratorBase> grasp_;
  std::string detector_plugin_name_;
  std::string localiser_plugin_name_;
  std::string grasping_plugin_name_;

  sh_base_template::DetectorBaseConfig detector_base_config_;
  std::vector<std::string> classes_;
  bool publish_detections_image_;

  rclcpp::Service<sh_interfaces::srv::RunPerception>::SharedPtr service_;
  rclcpp::Publisher<ImageMsg>::SharedPtr image_publisher_;

};

}  // namespace sh_perception_server

#endif  // SH_PERCEPTION_SERVER__PERCEPTION_SERVER_HPP_