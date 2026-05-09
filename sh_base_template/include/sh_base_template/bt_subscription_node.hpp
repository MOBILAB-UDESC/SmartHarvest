#ifndef SH_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_
#define SH_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_

#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sh_base_template
{
/**
 * @brief Non-blocking abstract class for managing a ROS 2 subscriber.
 *
 * @tparam NodeType ROS 2 node type (rclcpp::Node or rclcpp_lifecycle::LifecycleNode).
 * @tparam TopicType Message type.
 */
template <class NodeType, class TopicType>
class BTSubscriptionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for sh_base_template::BTSubscriptionNode class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit BTSubscriptionNode(
    const std::string& action_name,
    const BT::NodeConfig& node_config);

  /**
   * @brief A destructor for sh_base_template::BTSubscriptionNode class.
   */
  virtual ~BTSubscriptionNode()
  {}

  /**
   * @brief Final tick implementation.
   *
   * Main execution function required by BehaviorTree. Waits for a message to arrive
   * or the timeout expires.
   * On success, calls on_tick() function.
   * On timeout, calls on_timeout() function.
   *
   * @return BT::NodeStatus SUCCESS, FAILURE or RUNNING.
   */
  BT::NodeStatus tick() override final;

  /** Callback invoked by tick() when a new message is available.
   *
   * Custom application logic must be implemented here.
   * @param last_msg Latest message received.
   *
   * @return BT::NodeStatus SUCCESS, FAILURE or RUNNING.
   */
  virtual BT::NodeStatus on_tick(
    const std::shared_ptr<TopicType>& last_msg) = 0;

  /** Callback invoked by tick() when no message arrives on time.
   *
   * Custom application logic must be implemented here.
   *
   * @return BT::NodeStatus SUCCESS, FAILURE or RUNNING.
   */
  virtual BT::NodeStatus on_timeout() {
    RCLCPP_ERROR(logger_, "No message received in %.3f seconds.", sub_timeout_);
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief BehaviorTree halt callback.
   *
   * Clears buffered message state and resets BT status.
   */
  void halt() override
  {
    {
      std::lock_guard<std::mutex> lock(sub_mutex_);
      last_msg_.reset();
    }
    resetStatus();
  }

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

protected:
  // ROS 2 node members
  rclcpp::Logger logger_;
  typename NodeType::WeakPtr node_;

  /**
   * @brief Any subclass of BTSubscriptionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("topic_name", "Topic to subscribe to."),
      BT::InputPort<double>("sub_timeout"),
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

private:
  /**
   * @brief Subscriber initialization.
   *
   * @param topic_name Name of the topic to subscribe to.
   */
  void create_subscriber(const std::string& topic_name);

  void reset_timeout_deadline()
  {
    timeout_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(sub_timeout_));
  }

  // Executor settings
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Subscriber members
  typename rclcpp::Subscription<TopicType>::SharedPtr subscriber_;
  double sub_timeout_;
  std::chrono::steady_clock::time_point timeout_deadline_;

  std::mutex sub_mutex_;
  std::shared_ptr<TopicType> last_msg_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class NodeT, class TopicT>
inline BTSubscriptionNode<NodeT, TopicT>::BTSubscriptionNode(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
  BT::ActionNodeBase(action_name, node_config),
  logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Configuring.");

  // Blackboard/BT ports variables
  node_ = node_config.blackboard->get<typename NodeT::WeakPtr>("root_node");
  std::string topic_name;
  if (!getInput("topic_name", topic_name)) {
    RCLCPP_ERROR(logger_, "Missing input 'topic_name'.");
    throw std::invalid_argument("Missing input 'topic_name'");
  }
  if (!getInput("sub_timeout", sub_timeout_)) {
    RCLCPP_ERROR(logger_, "Missing value in port 'sub_timeout'.");
    throw std::invalid_argument("Missing input 'sub_timeout'");
  }

  create_subscriber(topic_name);

  RCLCPP_INFO(logger_, "Node created.");
}

template <class NodeT, class TopicT>
inline void BTSubscriptionNode<NodeT, TopicT>::create_subscriber(const std::string& topic_name)
{
  auto node = node_.lock();

  // Subscribers configuration
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  auto callback = [this](const std::shared_ptr<TopicT> msg) {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    last_msg_ = msg;
  };

  subscriber_ = node->template create_subscription<TopicT>(topic_name, 1, callback, sub_options);

  RCLCPP_INFO(logger_, "Subscription to %s topic created.", topic_name.c_str());
}

template <class NodeT, class TopicT>
inline BT::NodeStatus BTSubscriptionNode<NodeT, TopicT>::tick()
{
  if (!BT::isStatusActive(status())) {
    reset_timeout_deadline();
    setStatus(BT::NodeStatus::RUNNING);
  }

  executor_.spin_some();

  if (std::chrono::steady_clock::now() > timeout_deadline_) {
    auto status = on_timeout();
    reset_timeout_deadline();
    return status;
  }

  std::shared_ptr<TopicT> msg_copy;
  {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    if (!last_msg_) {
      return BT::NodeStatus::RUNNING;
    }
    msg_copy = last_msg_;
    last_msg_.reset();
  }

  auto status = on_tick(msg_copy);

  reset_timeout_deadline();

  return status;
}

}

#endif  // SH_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_
