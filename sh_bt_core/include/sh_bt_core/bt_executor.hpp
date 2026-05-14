#ifndef SH_BT_CORE__BT_EXECUTOR_HPP_
#define SH_BT_CORE__BT_EXECUTOR_HPP_

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_bt_core/bt_node_loader.hpp"
#include "sh_interfaces/srv/tick_tree.hpp"

namespace sh_bt_core
{

using CallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;

/**
 * @class sh_behavior_core::BTExecutor
 * @brief Lifecycle-manager that loads nodes and tree configuration and plugins
 * for smart harvesting operations.
 */
class BTExecutor : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief A constructor for sh_behavior_core::BTExecutor class.
   *
   * @param node_name Name of the node.
   */
  explicit BTExecutor(const std::string & node_name);


  /**
   * @brief A destructor for sh_behavior_core::BTExecutor class.
   */
  ~BTExecutor() = default;

private:
  /**
   * @brief Callback function for configure transition.
   *
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback function for activate transition.
   *
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for deactivate transition.
   *
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for cleanup transition.
   *
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  // CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for shutdown transition.
   *
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  // CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Declares all ROS2 parameters used by the BTExecutor node.
   */
  void declare_parameters();

  /**
   * @brief Creates and populates the BehaviorTree blackboard with all entries
   *        required by the BT nodes at runtime.
   *
   * @return BT::Blackboard::Ptr Fully populated blackboard.
   */
  BT::Blackboard::Ptr setup_blackboard();

  /**
   * @brief Callback function for ticking the Behavior Tree.
   */
  void tick_callback(const std::shared_ptr<sh_interfaces::srv::TickTree::Request> request,
  std::shared_ptr<sh_interfaces::srv::TickTree::Response> response);

  rclcpp::Logger logger_; // Ros 2 node logger
  rclcpp::Service<sh_interfaces::srv::TickTree>::SharedPtr tick_service_; // ROS 2 service

  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;

  BT::Tree tree_;
};

}  // namespace sh_bt_core


#endif  // SH_BT_CORE__BT_EXECUTOR_HPP_