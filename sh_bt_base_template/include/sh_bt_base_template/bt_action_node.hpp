#ifndef SH_BT_BASE_TEMPLATE__BT_ACTION_NODE_HPP_
#define SH_BT_BASE_TEMPLATE__BT_ACTION_NODE_HPP_

#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sh_interfaces/msg/error_codes.hpp"

namespace sh_bt_base_template
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
   * @brief A constructor for sh_bt_base_template::BTActionNode class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit BTActionNode(
    const std::string& action_name,
    const BT::NodeConfig& node_config);

  /**
   * @brief A destructor for sh_bt_base_template::BTActionNode class.
   */
  virtual ~BTActionNode()
  {}

  /**
   * @brief Final tick implementation.
   *
   * Main execution function required by BehaviorTree. Waits for an action response to arrive
   * or the timeout expires.
   * On SUCCESS, calls onTick() function.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus tick() override final;

  /**
   * @brief Method invoked by tick() to update the goal msg.
   *
   * @param goal_msg Mutable goal object to be filled by derived class.
   */
  virtual bool update_goal(std::shared_ptr<typename ActionType::Goal>& goal_msg) = 0;

  /** Callback invoked by tick() when action feedback is received.
   *
   * Implement a custom application logic here.
   * @param feedback Latest feedback message from the action server.
   */
  virtual void on_feedback(const std::shared_ptr<const typename ActionType::Feedback> feedback) = 0;

  /** Callback invoked by tick() when the action finishes with failure/canceled.
   *
   * Implement a custom application logic here.
   * @param result Wrapped action result.
   */
  virtual void on_failure(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& result) = 0;

  /** Callback invoked by tick() when the action exceeds configured response timeout.
   *
   * Implement a custom application logic here.
   */
  virtual void on_timeout() {}

  /** Callback invoked by tick() when the action finishes with success.
   *
   * Implement a custom application logic here.
   * @param result Wrapper for the action response.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus on_success(
    const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult& result) = 0;

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
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
   * @brief Initialize the action client.
   *
   * @param action_name Name of the action.
   */
  void create_action_client(const std::string& action_server_name);

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
  {}

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
  bool goal_result_received_;
  std::chrono::steady_clock::time_point timeout_deadline_;
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
  std::string action_server_name;
  getInput("action_name", action_server_name);
  getInput("action_response_timeout", action_response_timeout_);

  create_action_client(action_server_name);

  RCLCPP_INFO(logger_, "Node created.");
}

template <class NodeT, class ActionT>
inline void BTActionNode<NodeT, ActionT>::create_action_client(const std::string& action_server_name)
{
  auto node = node_.lock();

  // Action client configuration
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  client_ = rclcpp_action::create_client<ActionT>(node, action_server_name, callback_group_);

  if (!server_ready()) {
    RCLCPP_ERROR(logger_, "Action server '%s' not available.", action_server_name.c_str());
    throw std::runtime_error(
      std::string("Action server not available: ") + action_server_name);
  }
  RCLCPP_INFO(logger_, "Action server '%s' available.", action_server_name.c_str());

  // Initialize Action members.
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
  auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(logger_, "Goal rejected by server.");
    } else {
      RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result.");
    }
  };

  send_goal_options.feedback_callback = [this](
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
    const std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    on_feedback(feedback);
  };

  send_goal_options.result_callback =
    [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& result)
  {
    goal_result_received_ = true;
    result_ = result;
  };
  client_->async_send_goal(*goal_, send_goal_options);
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

#endif  // SH_BT_BASE_TEMPLATE__BT_ACTION_NODE_HPP_