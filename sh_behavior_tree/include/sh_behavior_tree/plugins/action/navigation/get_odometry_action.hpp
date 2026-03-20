#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__GET_ODOMETRY_ACTION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__GET_ODOMETRY_ACTION_HPP_

#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "sh_bt_base_template/bt_subscription_node.hpp"

namespace sh_behavior_tree
{

/**
 * @class sh_behavior_tree::GetOdometryAction
 * @brief BehaviorTree Action Node that acquires odometry data.
 */
class GetOdometryAction :
  public sh_bt_base_template::BTSubscriptionNode<rclcpp_lifecycle::LifecycleNode, nav_msgs::msg::Odometry>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::GetOdometryAction class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit GetOdometryAction(const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::GetOdometryAction class.
   */
  ~GetOdometryAction() = default;

  /**
   * @brief Creates list of BT ports.
   *
   * @return BT::PortsList List of ports with their type and description.
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Main execution function required by BehaviorTree.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus onTick(const std::shared_ptr<nav_msgs::msg::Odometry>& last_msg) override;

  double topic_sub_timeout_ = 0.1;

};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__GET_ODOMETRY_ACTION_HPP_