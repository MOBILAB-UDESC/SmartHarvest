#include "sh_behavior_tree/plugins/action/pick_and_place/move_gripper_action.hpp"

namespace sh_behavior_tree
{

MoveGripperAction::MoveGripperAction(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_bt_base_template::BTActionNode<
      rclcpp_lifecycle::LifecycleNode, MoveToNamedTargetAction>(action_name, node_config)
{}

MoveGripperAction::~MoveGripperAction()
{}

BT::PortsList MoveGripperAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("group_name", "Plannig group name."),
    BT::InputPort<std::string>("named_target", "Predifined NamedTarget from the SRDF."),
    BT::InputPort<bool>("attach", false, "Wheter to attach an object."),
    BT::InputPort<bool>("detach", false, "Wheter to detach an object."),
    BT::InputPort<std::string>("object_to_attach_detach", "", "Object name to attach/detach."),
    BT::InputPort<std::string>("link_to_attach_detach", "", "Link name to attach/detach."),
    BT::OutputPort<int>("error_code", "Error ID.")
  });
}

bool MoveGripperAction::update_goal(std::shared_ptr<typename MoveToNamedTargetAction::Goal>& goal_msg)
{
  if (!getInput("named_target", goal_msg->named_target)) {
    RCLCPP_ERROR(logger_, "Missing input 'named_target'.");
    return false;
  }
  if (!getInput("group_name", goal_msg->group_name)) {
    RCLCPP_ERROR(logger_, "Missing input 'group_name'.");
    return false;
  }
  getInput("attach", goal_msg->attach);
  getInput("detach", goal_msg->detach);
  getInput("object_to_attach_detach", goal_msg->object_to_attach_detach);
  getInput("link_to_attach_detach", goal_msg->link_to_attach_detach);
  return true;
}

void MoveGripperAction::on_feedback(
  const std::shared_ptr<const MoveToNamedTargetAction::Feedback> feedback)
{
  RCLCPP_INFO(logger_, "Feedback: %s", feedback->message.c_str());
}

BT::NodeStatus MoveGripperAction::on_success(
  const typename rclcpp_action::ClientGoalHandle<MoveToNamedTargetAction>::WrappedResult& result)
{
  RCLCPP_INFO(logger_, "Result is: %s", result.result->message.c_str());
  setOutput("error_code", sh_interfaces::msg::ErrorCodes::SUCCESS);
  return BT::NodeStatus::SUCCESS;
}

void MoveGripperAction::on_failure(
  const typename rclcpp_action::ClientGoalHandle<MoveToNamedTargetAction>::WrappedResult& result)
{
  RCLCPP_ERROR(logger_, result.result->message.c_str());
  setOutput("error_code", sh_interfaces::msg::ErrorCodes::PLANNING_FAILED);
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::MoveGripperAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::MoveGripperAction>(
    "MoveGripperAction", builder);
}