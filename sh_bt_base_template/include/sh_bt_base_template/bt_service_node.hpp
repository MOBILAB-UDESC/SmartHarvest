#ifndef SH_BT_BASE_TEMPLATE__BT_SERVICE_NODE_HPP_
#define SH_BT_BASE_TEMPLATE__BT_SERVICE_NODE_HPP_

#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "sh_interfaces/msg/error_codes.hpp"

namespace sh_bt_base_template
{
/**
 * @brief Abstract class for managing a ROS 2 service clients.
 *
 * @tparam NodeType ROS 2 node type (rclcpp::Node or rclcpp_lifecycle::LifecycleNode).
 * @tparam ServiceType Message type.
 */
template <class NodeType, class ServiceType>
class BTServiceNode : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for sh_bt_base_template::BTServiceNode class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit BTServiceNode(
    const std::string& action_name,
    const BT::NodeConfig& node_config);

  /**
   * @brief A destructor for sh_bt_base_template::BTServiceNode class.
   */
  virtual ~BTServiceNode()
  {}

  /**
   * @brief Final tick implementation.
   *
   * Main execution function required by BehaviorTree. Waits for a service response to arrive
   * or the timeout expires.
   * On SUCCESS, calls onTick() function.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus tick() override final;

  /**
   * @brief Method invoked by tick() to send a service request.
   *
   * @param request Service request.
   */
  virtual bool send_request(std::shared_ptr<typename ServiceType::Request>& request) = 0;

  /** Callback invoked by tick() when a service response arrives.
   *
   * Implement a custom application logic here with the response.
   * @param last_msg ROS 2 service response.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onTick(
    const std::shared_ptr<typename ServiceType::Response>& response) = 0;

protected:
  rclcpp::Logger logger_;

private:
  /**
   * @brief Initialized the service client.
   *
   * @param service_name Name of the service.
   */
  void create_service_client(const std::string& service_name);

  /**
   * @brief Checks if service server is available/ready.
   *
   * @return true or false.
   */
  bool service_ready()
  {
    if (!client_->wait_for_service(std::chrono::duration<double>(wait_for_service_timeout_))) {
      RCLCPP_ERROR(logger_, "Service server not available");
      return false;
    }
    return true;
  }

  // ROS 2 node members
  typename NodeType::WeakPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Service client members
  typename rclcpp::Client<ServiceType>::SharedPtr client_;
  std::shared_ptr<typename ServiceType::Request> request_;
  double service_response_timeout_;
  double wait_for_service_timeout_;
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class NodeT, class ServiceT>
inline BTServiceNode<NodeT, ServiceT>::BTServiceNode(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
  BT::SyncActionNode(action_name, node_config),
  logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Configuring.");

  // Blackboard/BT ports variables
  node_ = node_config.blackboard->get<typename NodeT::WeakPtr>("root_node");
  service_response_timeout_ = node_config.blackboard->get<double>("service_response_timeout");
  wait_for_service_timeout_ = node_config.blackboard->get<double>("wait_for_service_timeout");
  std::string service_name;
  getInput("service_name", service_name);

  create_service_client(service_name);

  RCLCPP_INFO(logger_, "Node created.");
}

template <class NodeT, class ServiceT>
inline void BTServiceNode<NodeT, ServiceT>::create_service_client(const std::string& service_name)
{
  auto node = node_.lock();

  // Service client configuration
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  rclcpp::QoS qos = rclcpp::QoS(1);

  client_ = node->template create_client<ServiceT>(service_name, qos, callback_group_);

  if (!service_ready()) {
    throw std::runtime_error("Service server not available.");
  }

  request_ = std::make_shared<typename ServiceT::Request>();

  RCLCPP_INFO(logger_, "Service client created : %s.", service_name.c_str());
}

template <class NodeT, class ServiceT>
inline BT::NodeStatus BTServiceNode<NodeT, ServiceT>::tick()
{
  if (!send_request(request_)) {
    return BT::NodeStatus::FAILURE;
  }

  auto future = client_->async_send_request(request_);

  if (executor_.spin_until_future_complete(
      future, std::chrono::duration<double>(service_response_timeout_))
      != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(logger_, "No response in %.3f seconds.", service_response_timeout_);
    return BT::NodeStatus::FAILURE;
  }

  auto result = future.get();
  auto status = onTick(result);

  // request_->reset();

  return status;
}

}

#endif  // SH_BT_BASE_TEMPLATE__BT_SERVICE_NODE_HPP_
