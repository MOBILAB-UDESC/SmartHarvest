#ifndef SH_BT_CORE__BT_EXECUTOR_HPP_
#define SH_BT_CORE__BT_EXECUTOR_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_bt_core/bt_node_loader.hpp"
#include "sh_interfaces/action/tick.hpp"

namespace sh_bt_core
{

/**
 * @class sh_bt_core::ActiveNodeTracker
 * @brief Behaviour tree node tracking.
 */
class ActiveNodeTracker : public BT::StatusChangeLogger
{
public:
  /**
   * @brief A constructor for sh_bt_core::ActiveNodeTracker class.
   *
   * @param root_node Root node of the tree to observe.
   */
  explicit ActiveNodeTracker(BT::TreeNode * root_node);

  /**
   * @brief Called by the tree on every status transition.
   *
   * @param timestamp Time of the transition.
   * @param node Node that changed status.
   * @param prev_status Status before the transition.
   * @param status Status after the transition.
   */
  void callback(
    BT::Duration timestamp,
    const BT::TreeNode & node,
    BT::NodeStatus prev_status,
    BT::NodeStatus status) override;

  void flush() override;

  /**
   * @brief Name of the last node seen in the RUNNING state.
   */
  std::string active_node();

private:
  std::mutex mutex_;
  std::string active_node_;
};

/**
 * @class sh_bt_core::BTExecutor
 * @brief Lifecycle-manager that loads BT nodes, tree configuration, and plugins
 * for smart harvesting operations.
 */
class BTExecutor : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;
  using TickAction = sh_interfaces::action::Tick;
  using GoalHandleTickAction = rclcpp_action::ServerGoalHandle<TickAction>;

  /**
   * @brief A constructor for sh_bt_core::BTExecutor class.
   *
   * @param node_name Name of the node.
   */
  explicit BTExecutor(const std::string & node_name);


  /**
   * @brief A destructor for sh_bt_core::BTExecutor class.
   */
  ~BTExecutor();

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
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for shutdown transition.
   *
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Declares all ROS2 parameters used by the BTExecutor node.
   */
  void declare_parameters();

  /**
   * @brief Creates and populates the BehaviorTree blackboard with all entries
   * required by the BT nodes at runtime.
   *
   * @return BT::Blackboard::Ptr Fully populated blackboard.
   */
  BT::Blackboard::Ptr setup_blackboard();

  /**
   * @brief Creates the tick action server.
   */
  void setup_tick_action();

  /**
   * @brief Runs the BT ticking loop.
   *
   * @param goal_handle Handle of accepted goal.
   */
  void execute_tick(const std::shared_ptr<GoalHandleTickAction> & goal_handle);

  /**
   * @brief Request the running tick to stop, then wait for it to finish.
   */
  void stop_running_goal();

  rclcpp::Logger logger_; // Ros 2 node logger
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_;

  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;

  rclcpp_action::Server<TickAction>::SharedPtr tick_action_server_;
  std::atomic_bool goal_running_{false};
  std::thread tick_thread_;
  std::shared_ptr<std::atomic_bool> cancel_token_{std::make_shared<std::atomic_bool>(false)};

  double tick_period_s_{0.01};
  double feedback_period_s_{0.2};
};

}  // namespace sh_bt_core


#endif  // SH_BT_CORE__BT_EXECUTOR_HPP_