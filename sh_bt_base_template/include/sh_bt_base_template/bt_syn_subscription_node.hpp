#ifndef SH_BT_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_
#define SH_BT_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_

#include <functional>
#include <tuple>
#include <utility>

#include "behaviortree_cpp/action_node.h"
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_epsilon_time.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sh_interfaces/msg/error_codes.hpp"

namespace sh_bt_base_template
{
/**
 * @brief Abstract base class for Behavior Tree nodes
 *        that require synchronized ROS 2 topic subscriptions.
 *
 * @tparam NodeType ROS 2 node type (rclcpp::Node or rclcpp_lifecycle::LifecycleNode).
 * @tparam MessageTypes Parameter pack of ROS 2 messages types to synchronize (2-9 types).
 *         Must match topic count.
 *
 * @example
 *    class GetCameraDataAction : public BTSyncSubscriptionNode<
 *        rclcpp_lifecycle::LyfecycleNode,
 *        sensor_msgs::msg::Image,
 *        sensor_msgs::msg::Image,
 *        sensor_msgs::msg::CameraInfo>
 */
template <class NodeType, class... MessageTypes>
class BTSyncSubscriptionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for sh_bt_base_template::BTSyncSubscriptionNode class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit BTSyncSubscriptionNode(
    const std::string& action_name,
    const BT::NodeConfig& node_config);

  /**
   * @brief A destructor for sh_bt_base_template::BTSyncSubscriptionNode class.
   */
  virtual ~BTSyncSubscriptionNode()
  {}

  /**
   * @brief Final tick implementation.
   *
   * Main execution function required by BehaviorTree. Waits for synchronized messages to arrive
   * or the timeout expires.
   * On SUCCESS, calls onTick() function.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus tick() override final;

  /** Callback invoked by tick() when synchronized messages arrive.
   *
   * Implement a custom application logic here with the latest synchronized messages.
   * @param last_msgs synchronized messages in the order of MessageTypes.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onTick(
    const std::shared_ptr<const MessageTypes>&... last_msgs) = 0;

  /** Callback invoked by tick() when the subscriber exceeds configured response timeout.
   *
   * Implement a custom application logic here.
   */
  virtual void on_timeout() {}

  /**
   * @brief BehaviorTree halt callback.
   */
  void halt() override
  {
    {
      std::lock_guard<std::mutex> lock(sub_mutex_);
      msg_received_ = false;
      last_msgs_ = {};
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
  using SyncPolicy = message_filters::sync_policies::ApproximateEpsilonTime<MessageTypes...>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  /**
   * @brief Any subclass of BTSyncSubscriptionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::vector<std::string>>(
        "topic_names", "Topics to receive synchronized msgs."),
      BT::InputPort<double>("sync_timeout"),
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  // ROS 2 members
  rclcpp::Logger logger_;

private:
  /**
   * @brief Subscriber creation, synchronizer initialization and callback registration.
   *
   * @tparam IndexSequence Compile-time integer sequence (0 to N-1).
   * @param topic_names List of N topic names (ordered)
   */
  template <std::size_t... IndexSequence>
  void create_subscribers(
    const std::vector<std::string>& topic_names,
    std::index_sequence<IndexSequence...>);

  /**
   * @brief Initialized a single subscriber for every messages types
   *
   * @tparam Index Tuple index.
   * @param parent Shared pointer to a ROS 2 node.
   * @param options Subcription options.
   * @param qos Quality of service configuration.
   * @param topic_name Name of the topic.
   */
  template <std::size_t Index>
  void create_single_subscriber(
    const typename NodeType::SharedPtr& parent,
    const rclcpp::SubscriptionOptions& options,
    const rclcpp::QoS& qos,
    const std::string& topic_name);

  /**
   * @brief Callback invoked when synchronized messages arrive.
   *
   * @param msgs Synchronized messages (one per MessageType, in order).
   */
  void msg_callback(const std::shared_ptr<const MessageTypes>&... msgs);

  /**
   * @brief Compile-time mapping of the placeholders for std_bind.
   *
   * @tparam N Placeholder index from 0 to 8 (e.g. N=2 -> _3).
   */
  template <std::size_t N>
  struct placeholder_at {
    static auto& get() {
      static const auto placeholders = std::make_tuple(
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4,
        std::placeholders::_5,
        std::placeholders::_6,
        std::placeholders::_7,
        std::placeholders::_8,
        std::placeholders::_9
      );
      return std::get<N>(placeholders);
    }
  };

  // ROS 2 node members
  typename NodeType::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Synchronized messages members
  double sync_timeout_;
  std::tuple<message_filters::Subscriber<MessageTypes, NodeType>...> subscribers_;
  std::unique_ptr<Synchronizer> synchronizer_;
  std::tuple<std::shared_ptr<const MessageTypes>...> last_msgs_;
  bool msg_received_;
  std::mutex sub_mutex_;
  std::chrono::steady_clock::time_point timeout_deadline_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class NodeT, class... MessageTs>
inline BTSyncSubscriptionNode<NodeT, MessageTs...>::BTSyncSubscriptionNode(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
  BT::ActionNodeBase(action_name, node_config),
  logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Configuring synchronized subscribers.");

  // Blackboard/BT ports variables
  node_ = node_config.blackboard->get<typename NodeT::WeakPtr>("root_node");
  std::vector<std::string> topic_names;
  getInput("topic_names", topic_names);
  getInput("sync_timeout", sync_timeout_);
  msg_received_ = false;

  // Compile-time checker
  static_assert(sizeof...(MessageTs) >= 2 && sizeof...(MessageTs) <= 9,
    "BTSyncSubscriptionNode requires between 2 and 9 message types.");

  // Run-time checker
  if (topic_names.size() != sizeof...(MessageTs)) {
    RCLCPP_ERROR(logger_, "Topic names count mismatch for node '%s': expected %zu topics"
      " but got %zu in the tree.xml.",
      action_name.c_str(), sizeof...(MessageTs), topic_names.size());
    throw std::invalid_argument("Topic names count mismatch.");
  }

  // Synchronized subcribers
  create_subscribers(topic_names, std::index_sequence_for<MessageTs...>{});

  RCLCPP_INFO(logger_, "Node created.");
}

template <class NodeT, class... MessageTs>
template <std::size_t... Is>
inline void BTSyncSubscriptionNode<NodeT, MessageTs...>::create_subscribers(
  const std::vector<std::string>& topic_names,
  std::index_sequence<Is...>)
{
  if (node_.expired()) {
    throw std::invalid_argument("Node pointer is empty.");
  }
  auto node = node_.lock();

  // Subscribers configuration
  callback_group_ =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  rclcpp::QoS qos = rclcpp::QoS(1);

  // Fold expression to extend the function for all N subscribers
  (create_single_subscriber<Is>(node, sub_options, qos, topic_names[Is]), ...);

  // Synchronizer setup
  synchronizer_ = std::make_unique<Synchronizer>(
    SyncPolicy(1, rclcpp::Duration::from_seconds(0.1f)),
    std::get<Is>(subscribers_)...);

  synchronizer_->registerCallback(
    std::bind(
      &BTSyncSubscriptionNode::msg_callback,
      this,
      placeholder_at<Is>::get()...));
}

template <class NodeT, class... MessageTs>
template <std::size_t I>
inline void BTSyncSubscriptionNode<NodeT, MessageTs...>::create_single_subscriber(
  const typename NodeT::SharedPtr& parent,
  const rclcpp::SubscriptionOptions& options,
  const rclcpp::QoS& qos,
  const std::string& topic_name)
{
  std::get<I>(subscribers_).subscribe(
    parent, topic_name, qos.get_rmw_qos_profile(), options);

  RCLCPP_INFO(logger_, "Subscription to %s topic created.", topic_name.c_str());
}

template <class NodeT, class... MessageTs>
inline void BTSyncSubscriptionNode<NodeT, MessageTs...>::msg_callback(
  const std::shared_ptr<const MessageTs>&... msgs)
{
  std::lock_guard<std::mutex> lock(sub_mutex_);
  msg_received_ = true;
  last_msgs_ = std::make_tuple(msgs...);
}

template <class NodeT, class... MessageTs>
inline BT::NodeStatus BTSyncSubscriptionNode<NodeT, MessageTs...>::tick()
{
  if (!BT::isStatusActive(status())) {
    timeout_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(sync_timeout_));
    setStatus(BT::NodeStatus::RUNNING);
  }

  executor_.spin_some();

  if (std::chrono::steady_clock::now() > timeout_deadline_) {
    on_timeout();
    RCLCPP_ERROR(logger_, "No message received in %.3f seconds.", sync_timeout_);
    return BT::NodeStatus::FAILURE;
  }

  std::tuple<std::shared_ptr<const MessageTs>...> msgs_copy;
  {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    if (!msg_received_) {
      return BT::NodeStatus::RUNNING;
    }
    msgs_copy = last_msgs_;
    last_msgs_ = {};
    msg_received_ = false;
  }

  auto status = std::apply(
    [this](const auto&... msgs) {
      return onTick(msgs...);
    },
    msgs_copy);

  return status;
}

}

#endif  // SH_BT_BASE_TEMPLATE__BT_SUBSCRIPTION_NODE_HPP_
