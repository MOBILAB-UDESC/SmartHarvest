#ifndef SH_BASE_TEMPLATE__BT_ACTION_NODE_HPP_
#define SH_BASE_TEMPLATE__BT_ACTION_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sh_interfaces/msg/error_codes.hpp"

namespace sh_base_template
{
/**
 * @brief Abstract class for managing ROS 2 action clients.
 *
 * @tparam NodeType ROS 2 node type (rclcpp::Node or rclcpp_lifecycle::LifecycleNode).
 * @tparam ActionType ROS 2 action type.
 */
template <class NodeType, class ActionType>
class BTActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for sh_base_template::BTActionNode class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit BTActionNode(
    const std::string & action_name,
    const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_base_template::BTActionNode class.
   */
  virtual ~BTActionNode()
  {
    cancel_active_goal();
  }

  /**
   * @brief Final tick implementation.
   *
   * Main execution function required by BehaviorTree. Waits for an action response to arrive
   * or the timeout expires.
   * On SUCCESS, calls onTick() function.
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus tick() override final;

  /**
   * @brief Method invoked by tick() to update the goal msg.
   *
   * @param goal_msg Mutable goal object to be filled by derived class.
   */
  virtual bool update_goal(std::shared_ptr<typename ActionType::Goal> & goal_msg) = 0;

  /**
   * @brief Callback invoked by tick() when action feedback is received.
   *
   * Implement a custom application logic here.
   * @param feedback Latest feedback message from the action server.
   */
  virtual void on_feedback(const std::shared_ptr<const typename ActionType::Feedback> feedback) = 0;

  /**
   * @brief Callback invoked by tick() when the action finishes with failure/canceled.
   *
   * Implement a custom application logic here.
   * @param result Wrapped action result.
   */
  virtual void on_failure(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult & result) = 0;

  /**
   * @brief Callback invoked by tick() when the action exceeds configured response timeout.
   *
   * Implement a custom application logic here.
   */
  virtual void on_timeout() {}

  /**
   * @brief Callback invoked by tick() when the action finishes with success.
   *
   * Implement a custom application logic here.
   * @param result Wrapper for the action response.
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus on_success(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult & result) = 0;

  /**
   * @brief Creates list of BT ports.
   *
   * @return PortsList Containing basic ports along with node-specific ports.
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  rclcpp::Logger logger_;

  /**
   * @brief Any subclass of BTActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("action_name"),
      BT::InputPort<double>("action_response_timeout"),
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

private:
  /**
   * @brief Initialise the action client.
   *
   * @param action_name Name of the action.
   */
  void create_action_client();

  /**
   * @brief Check if action server is available/ready.
   *
   * @return true or false.
   */
  bool server_ready()
  {
    if (!client_->wait_for_action_server(std::chrono::duration<double>(wait_for_action_timeout_))) {
      return false;
    }
    return true;
  }

  /**
   * @brief BehaviorTree halt callback.
   */
  void halt() override
  {
    cancel_active_goal();
    resetStatus();
  }

  /**
   * @brief Ask the server to cancel the goal currently in flight, if any.
   *
   * @return true or false.
   */
  bool cancel_active_goal();

  /**
   * @brief Send one goal asynchronously and register goal/feedback/result callbacks.
   */
  void send_a_goal();

  // ROS 2 node members
  typename NodeType::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Action client members
  typename rclcpp_action::Client<ActionType>::SharedPtr client_;
  std::shared_ptr<typename ActionType::Goal> goal_;
  typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult result_;
  double action_response_timeout_;
  double wait_for_action_timeout_;
  double cancel_timeout_;
  bool goal_result_received_;
  std::chrono::steady_clock::time_point timeout_deadline_;

  typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr goal_handle_;
  bool goal_running_{false};
  std::string action_server_name_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class NodeT, class ActionT>
inline BTActionNode<NodeT, ActionT>::BTActionNode(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
  BT::ActionNodeBase(action_name, node_config),
  logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Configuring.");

  // Blackboard/BT ports variables
  node_ = node_config.blackboard->get<typename NodeT::WeakPtr>("root_node");
  wait_for_action_timeout_ = node_config.blackboard->get<double>("wait_for_action_timeout");
  cancel_timeout_ = node_config.blackboard->get<double>("default_cancel_timeout");

  getInput("action_name", action_server_name_);
  getInput("action_response_timeout", action_response_timeout_);

  create_action_client();

  RCLCPP_INFO(logger_, "Node created.");
}

template <class NodeT, class ActionT>
inline void BTActionNode<NodeT, ActionT>::create_action_client()
{
  auto node = node_.lock();

  // Action client configuration
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  client_ = rclcpp_action::create_client<ActionT>(node, action_server_name_, callback_group_);

  if (!server_ready()) {
    RCLCPP_ERROR(logger_, "Action server '%s' not available.", action_server_name_.c_str());
    throw std::runtime_error(
      std::string("Action server not available: ") + action_server_name_);
  }
  RCLCPP_INFO(logger_, "Action server '%s' available.", action_server_name_.c_str());

  // Initialise Action members.
  goal_ = std::make_shared<typename ActionT::Goal>();
  result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

  RCLCPP_INFO(logger_, "Action client created.");
}

template <class NodeT, class ActionT>
void BTActionNode<NodeT, ActionT>::send_a_goal()
{
  timeout_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(action_response_timeout_));

  goal_result_received_ = false;
  goal_handle_.reset();
  goal_running_ = true;

  auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(logger_, "Goal rejected by server.");
      goal_running_ = false;
      return;
    }
    RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result.");
    goal_handle_ = goal_handle;
  };

  send_goal_options.feedback_callback = [this](
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
    const std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    on_feedback(feedback);
  };

  send_goal_options.result_callback =
    [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
  {
    goal_result_received_ = true;
    result_ = result;
    goal_handle_.reset();
    goal_running_ = false;
  };
  client_->async_send_goal(*goal_, send_goal_options);
}

template <class NodeT, class ActionT>
inline bool BTActionNode<NodeT, ActionT>::cancel_active_goal()
{
  auto in_flight = [this]() {
    return goal_running_ && !goal_result_received_;
  };

  if (!client_ || !in_flight()) {
    goal_handle_.reset();
    return false;
  }

  RCLCPP_WARN(
    logger_, "Cancelling goal on '%s'.", action_server_name_.c_str());

  const auto cancel_deadline =
    std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(cancel_timeout_));

  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle;
  {
    while (std::chrono::steady_clock::now() < cancel_deadline) {
      handle = goal_handle_;
      if (handle || goal_result_received_ || !goal_running_) {
        break;
      }
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  bool acknowledged = false;
  auto cancel_future = handle
    ? client_->async_cancel_goal(handle)
    : client_->async_cancel_all_goals();

  if (executor_.spin_until_future_complete(
        cancel_future, cancel_deadline - std::chrono::steady_clock::now())
      == rclcpp::FutureReturnCode::SUCCESS)
  {
    acknowledged = true;
    RCLCPP_INFO(logger_, "Cancel acknowledged by '%s'.", action_server_name_.c_str());
  } else {
    RCLCPP_ERROR(
      logger_, "No cancel response from '%s' in %.3f seconds.",
      action_server_name_.c_str(), cancel_timeout_);
  }

  if (acknowledged) {
    while (std::chrono::steady_clock::now() < cancel_deadline) {
      executor_.spin_some();
      if (goal_result_received_) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (goal_result_received_) {
      RCLCPP_INFO(logger_, "Cancel goal on '%s' terminated.", action_server_name_.c_str());
    } else {
      RCLCPP_WARN(
        logger_, "'%s' accepted the cancel but did not terminate the goal in %.3f s.",
        action_server_name_.c_str(), cancel_timeout_);
    }
  }

  goal_handle_.reset();
  goal_running_ = false;
  return acknowledged;
}

template <class NodeT, class ActionT>
inline BT::NodeStatus BTActionNode<NodeT, ActionT>::tick()
{
  if (!BT::isStatusActive(status())) {
    if (!update_goal(goal_)) {
      return BT::NodeStatus::FAILURE;
    }
    send_a_goal();
    setStatus(BT::NodeStatus::RUNNING);
  }

  executor_.spin_some();

  if (action_response_timeout_ > 0.0 && std::chrono::steady_clock::now() > timeout_deadline_) {
    cancel_active_goal();
    on_timeout();
    RCLCPP_ERROR(logger_, "No response received in %.3f seconds.", action_response_timeout_);
    setStatus(BT::NodeStatus::FAILURE);
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_result_received_) {
    return BT::NodeStatus::RUNNING;
  } else {
    if (result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
      auto status = on_success(result_);
      return status;
    }
    on_failure(result_);
    setStatus(BT::NodeStatus::FAILURE);
    return BT::NodeStatus::FAILURE;
  }
}

}

#endif  // SH_BASE_TEMPLATE__BT_ACTION_NODE_HPP_