#ifndef SH_BT_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_
#define SH_BT_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_

#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sh_bt_base_template
{
/**
 * @brief Abstract class for managing a ROS 2 subscriber.
 *
 * @tparam NodeType ROS 2 node type (rclcpp::Node or rclcpp_lifecycle::LifecycleNode).
 * @tparam TopicType Message type.
 */
template <class NodeType, class TopicType>
class BTSubscriptionNode : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for sh_bt_base_template::BTSubscriptionNode class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit BTSubscriptionNode(
    const std::string& action_name,
    const BT::NodeConfig& node_config);

  /**
   * @brief A destructor for sh_bt_base_template::BTSubscriptionNode class.
   */
  virtual ~BTSubscriptionNode()
  {}

  /**
   * @brief Final tick implementation.
   *
   * Main execution function required by BehaviorTree. Waits for a message to arrive
   * or the timeout expires.
   * On SUCCESS, calls onTick() function.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus tick() override final;

  /** Callback invoked by tick() when a message arrives.
   *
   * Implement a custom application logic here with the latest message.
   * @param last_msg ROS 2 topic message.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onTick(
    const std::shared_ptr<TopicType>& last_msg) = 0;

  /**
   * @brief Controls whether the last received message set is retained after a tick.
   *
   * @return false by default. Override to return true if needed.
   */
  virtual bool keep_last_message() const
  {
    return false;
  }

protected:
  rclcpp::Logger logger_;

private:
  /**
   * @brief Initialized the subscriber.
   *
   * @param topic_name Name of the topic.
   */
  void create_subscriber(const std::string& topic_name);

  // ROS 2 node members
  typename NodeType::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Subscriber members
  typename rclcpp::Subscription<TopicType>::SharedPtr subscriber_;
  std::mutex sub_mutex_;
  std::shared_ptr<TopicType> last_msg_;
  double wait_for_msg_timeout_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class NodeT, class TopicT>
inline BTSubscriptionNode<NodeT, TopicT>::BTSubscriptionNode(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
  BT::SyncActionNode(action_name, node_config),
  logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Configuring.");

  // Blackboard/BT ports variables
  node_ = node_config.blackboard->get<typename NodeT::WeakPtr>("root_node");
  wait_for_msg_timeout_ = node_config.blackboard->get<double>("wait_for_msg_timeout");
  std::string topic_name;
  getInput("topic_name", topic_name);

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
  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::duration<double>(wait_for_msg_timeout_);
  std::shared_ptr<TopicT> msg;
  while (true) {
    executor_.spin_some();
    {
      std::lock_guard<std::mutex> lock(sub_mutex_);
      msg = last_msg_;
    }
    if (msg) {
      break;
    }

    if (std::chrono::steady_clock::now() - start >= timeout) {
      RCLCPP_INFO(logger_, "No msg arrived in %.3f seconds.", timeout.count());
      return BT::NodeStatus::FAILURE;
    }
  }

  auto status = onTick(msg);

  if (!keep_last_message()) {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    last_msg_.reset();
  }

  return status;
}

}

#endif  // SH_BT_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_
