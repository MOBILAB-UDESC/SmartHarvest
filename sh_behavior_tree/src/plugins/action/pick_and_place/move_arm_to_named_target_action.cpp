#include "sh_behavior_tree/plugins/action/pick_and_place/move_arm_to_named_target_action.hpp"

namespace sh_behavior_tree
{

MoveArmToNamedTargetAction::MoveArmToNamedTargetAction(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_bt_base_template::BTActionNode<
      rclcpp_lifecycle::LifecycleNode, MoveToNamedTargetAction>(action_name, node_config)
{}

MoveArmToNamedTargetAction::~MoveArmToNamedTargetAction()
{}

BT::PortsList MoveArmToNamedTargetAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "Action name."),
    BT::InputPort<std::string>("group_name", "Plannig group name."),
    BT::InputPort<std::string>("named_target", "Predifined NamedTarget from the SRDF."),
    BT::OutputPort<int>("error_code", "Error ID.")
  };
}

bool MoveArmToNamedTargetAction::update_goal(std::shared_ptr<typename MoveToNamedTargetAction::Goal>& goal_msg)
{
  if (!getInput("named_target", goal_msg->named_target)) {
    RCLCPP_ERROR(logger_, "Missing input 'named_target'.");
    return false;
  }
  if (!getInput("group_name", goal_msg->group_name)) {
    RCLCPP_ERROR(logger_, "Missing input 'group_name'.");
    return false;
  }
  return true;
}

void MoveArmToNamedTargetAction::on_feedback(
  const std::shared_ptr<const MoveToNamedTargetAction::Feedback> feedback)
{
  RCLCPP_INFO(logger_, "Feedback: %s", feedback->message.c_str());
}

BT::NodeStatus MoveArmToNamedTargetAction::on_success(
  const typename rclcpp_action::ClientGoalHandle<MoveToNamedTargetAction>::WrappedResult& result)
{
  RCLCPP_INFO(logger_, "Result is: %s", result.result->message.c_str());
  setStatus(BT::NodeStatus::SUCCESS);
  setOutput("error_code", sh_interfaces::msg::ErrorCodes::SUCCESS);
  return BT::NodeStatus::SUCCESS;
}

void MoveArmToNamedTargetAction::on_failure(
  const typename rclcpp_action::ClientGoalHandle<MoveToNamedTargetAction>::WrappedResult& result)
{
  RCLCPP_ERROR(logger_, result.result->message.c_str());
  setOutput("error_code", sh_interfaces::msg::ErrorCodes::PLANNING_FAILED);
  setStatus(BT::NodeStatus::FAILURE);
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::MoveArmToNamedTargetAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::MoveArmToNamedTargetAction>(
    "MoveArmToNamedTargetAction", builder);
}