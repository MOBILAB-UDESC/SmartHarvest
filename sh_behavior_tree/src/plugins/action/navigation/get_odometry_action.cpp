#include "sh_behavior_tree/plugins/action/navigation/get_odometry_action.hpp"

namespace sh_behavior_tree
{

GetOdometryAction::GetOdometryAction(const std::string & action_name, const BT::NodeConfig & node_config) :
  sh_bt_base_template::BTSubscriptionNode<rclcpp_lifecycle::LifecycleNode, nav_msgs::msg::Odometry>(action_name, node_config)
{}

BT::PortsList GetOdometryAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "Topic to receive a msg")
  };
}

BT::NodeStatus GetOdometryAction::onTick(const std::shared_ptr<nav_msgs::msg::Odometry>& last_msg)
{
  std::cout << "MSG: " << last_msg->pose.pose.position.x << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::GetOdometryAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::GetOdometryAction>("GetOdometryAction", builder);
}