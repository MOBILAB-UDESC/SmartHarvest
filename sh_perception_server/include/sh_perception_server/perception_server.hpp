#ifndef SH_PERCEPTION_SERVER__PERCEPTION_SERVER_HPP_
#define SH_PERCEPTION_SERVER__PERCEPTION_SERVER_HPP_

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/perception_pipeline_base.hpp"
#include "sh_interfaces/srv/run_perception.hpp"

namespace sh_perception_server
{

using CallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;

/**
 * @class sh_perception_server::PerceptionServer
 * @brief Lifecycle-manager that loads a perception pipeline plugin and handles perception requests.
 */
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
  ~PerceptionServer();

protected:
  /**
   * @brief Callback function for configure transition.
   *
   * Loads and configures the perception pipeline plugin.
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback function for activate transition.
   *
   * Creates a service for perception task.
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override;

  /**
   * @brief Callback function for deactivate transition.

  * @param state A reference to the state of the Lifecycle Node.
  * @return SUCCESS or FAILURE.
  */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override;

  /**
   * @brief Callback function for cleanup transition.

  * @param state A reference to the state of the Lifecycle Node.
  * @return SUCCESS or FAILURE.
  */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override;

  /**
   * @brief Callback function for shutdown transition.

  * @param state A reference to the state of the Lifecycle Node.
  * @return SUCCESS or FAILURE.
  */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override;

private:
  /**
   * @brief Handle perception service requests.
   *
   * Executes the perception pipeline and populates the response with estimated object poses.
   * @param request Service request containing Perception input data.
   * @param response Service response to populate with perception results.
   */
  void trigger_perception_callback(
    const std::shared_ptr<sh_interfaces::srv::RunPerception::Request> request,
    std::shared_ptr<sh_interfaces::srv::RunPerception::Response> response);

  pluginlib::ClassLoader<sh_base_template::PerceptionPipelineBase> pipeline_loader_;
  std::string pipeline_plugin_name_;
  pluginlib::UniquePtr<sh_base_template::PerceptionPipelineBase> pipeline_;

  rclcpp::Service<sh_interfaces::srv::RunPerception>::SharedPtr service_;
};

}  // namespace sh_perception_server

#endif  // SH_PERCEPTION_SERVER__PERCEPTION_SERVER_HPP_