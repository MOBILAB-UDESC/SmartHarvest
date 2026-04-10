#include "sh_behavior_tree/plugins/action/navigation/move_base_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace sh_behavior_tree
{

MoveBaseAction::MoveBaseAction(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_bt_base_template::BTActionNode<
      rclcpp_lifecycle::LifecycleNode, NavigateToPose>(action_name, node_config)
{
  node_ = node_config.blackboard->get<typename rclcpp_lifecycle::LifecycleNode::WeakPtr>("root_node");
}

MoveBaseAction::~MoveBaseAction()
{}

BT::PortsList MoveBaseAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose")
  });
}

bool MoveBaseAction::update_goal(std::shared_ptr<NavigateToPose::Goal>& goal_msg)
{
  if (!getInput("goal_pose", goal_msg->pose)) {
    RCLCPP_ERROR(logger_, "Input port 'goal_pose' is empty.");
    return false;
  }
  return true;
}

void MoveBaseAction::on_feedback(
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(logger_, "Distance remaining: %.3f", feedback->distance_remaining);
}

BT::NodeStatus MoveBaseAction::on_success(
  const GoalHandleNavigateToPose::WrappedResult& result)
{
  RCLCPP_INFO(logger_, "Result is: %s", result.result->error_msg.c_str());
  return BT::NodeStatus::SUCCESS;
}

void MoveBaseAction::on_failure(
  const GoalHandleNavigateToPose::WrappedResult& result)
{
  RCLCPP_ERROR(logger_, "Result is: %s", result.result->error_msg.c_str());
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::MoveBaseAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::MoveBaseAction>(
    "MoveBaseAction", builder);
}