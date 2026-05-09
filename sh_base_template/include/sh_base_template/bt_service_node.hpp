#ifndef SH_BASE_TEMPLATE__BT_SERVICE_NODE_HPP_
#define SH_BASE_TEMPLATE__BT_SERVICE_NODE_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "sh_interfaces/msg/error_codes.hpp"

namespace sh_base_template
{
/**
 * @brief Abstract class for managing a ROS 2 service client.
 *
 * @tparam NodeType ROS 2 node type (rclcpp::Node or rclcpp_lifecycle::LifecycleNode).
 * @tparam ServiceType Message type.
 *
 * @example
 *    class PredictAction : public BTServiceNode<
 *        rclcpp_lifecycle::LyfecycleNode,
 *        sh_interfaces::srv::DetectObjects>
 */
template <class NodeType, class ServiceType>
class BTServiceNode : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for sh_base_template::BTServiceNode class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit BTServiceNode(
    const std::string& action_name,
    const BT::NodeConfig& node_config);

  /**
   * @brief A destructor for sh_base_template::BTServiceNode class.
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

  /** Callback invoked by tick() when the service exceeds configured response timeout.
   *
   * Implement a custom application logic here.
   */
  virtual void on_timeout() {}

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
   * @brief Any subclass of BTServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name"),
      BT::InputPort<double>("service_response_timeout"),
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

private:
  /**
   * @brief Initializes the service client.
   *
   * @param service_name Name of the service.
   */
  void create_service_client(const std::string& service_name);

  /**
   * @brief Checks if service server is available/ready.
   *
   * @return true or false.
   */
  bool service_ready(const std::string& service_name)
  {
    if (!client_->wait_for_service(std::chrono::duration<double>(wait_for_service_timeout_))) {
      RCLCPP_ERROR(logger_, "Service server '%s' not available in %.3f seconds.",
        service_name.c_str(), wait_for_service_timeout_);
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
  wait_for_service_timeout_ = node_config.blackboard->get<double>("wait_for_service_timeout");
  std::string service_name;
  getInput("service_name", service_name);
  getInput("service_response_timeout", service_response_timeout_);

  create_service_client(service_name);

  RCLCPP_INFO(logger_, "Node created.");
}

template <class NodeT, class ServiceT>
inline void BTServiceNode<NodeT, ServiceT>::create_service_client(const std::string& service_name)
{
  if (node_.expired()) {
    throw std::invalid_argument("Root node pointer is expired.");
  }
  auto node = node_.lock();

  // Service client configuration
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(callback_group_, node->get_node_base_interface());
  rclcpp::QoS qos = rclcpp::QoS(1);

  client_ = node->template create_client<ServiceT>(service_name, qos, callback_group_);

  if (!service_ready(service_name)) {
    throw std::runtime_error("Service server not available.");
  }

  request_ = std::make_shared<typename ServiceT::Request>();

  RCLCPP_INFO(logger_, "Service client created for '%s'.", service_name.c_str());
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
    on_timeout();
    RCLCPP_WARN(logger_, "No response in %.3f seconds.", service_response_timeout_);
    return BT::NodeStatus::FAILURE;
  }

  auto result = future.get();
  auto status = onTick(result);

  return status;
}

}

#endif  // SH_BASE_TEMPLATE__BT_SERVICE_NODE_HPP_
