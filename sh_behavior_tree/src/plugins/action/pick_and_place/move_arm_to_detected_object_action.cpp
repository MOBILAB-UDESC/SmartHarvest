#include "sh_behavior_tree/plugins/action/pick_and_place/move_arm_to_detected_object_action.hpp"

namespace sh_behavior_tree
{

MoveArmToDetectedObjectAction::MoveArmToDetectedObjectAction(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_base_template::BTActionNode<
      rclcpp_lifecycle::LifecycleNode, MoveToObjectAction>(action_name, node_config)
{}

MoveArmToDetectedObjectAction::~MoveArmToDetectedObjectAction()
{}

BT::PortsList MoveArmToDetectedObjectAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("group_name", "Plannig group name."),
    BT::InputPort<std::string>("object_name", "Object in the planning scene."),
    BT::OutputPort<int>("error_code", "Error ID.")
  });
}

bool MoveArmToDetectedObjectAction::update_goal(std::shared_ptr<typename MoveToObjectAction::Goal>& goal_msg)
{
  if (!getInput("object_name", goal_msg->object_name)) {
    RCLCPP_ERROR(logger_, "Missing input 'object_name'.");
    return false;
  }
  if (!getInput("group_name", goal_msg->group_name)) {
    RCLCPP_ERROR(logger_, "Missing input 'group_name'.");
    return false;
  }

  return true;
}

void MoveArmToDetectedObjectAction::on_feedback(
  const std::shared_ptr<const MoveToObjectAction::Feedback> feedback)
{
  RCLCPP_INFO(logger_, "Feedback: %s", feedback->message.c_str());
}

BT::NodeStatus MoveArmToDetectedObjectAction::on_success(
  const typename rclcpp_action::ClientGoalHandle<MoveToObjectAction>::WrappedResult& result)
{
  RCLCPP_INFO(logger_, "Result is: %s", result.result->message.c_str());
  setStatus(BT::NodeStatus::SUCCESS);
  setOutput("error_code", sh_interfaces::msg::ErrorCodes::SUCCESS);
  return BT::NodeStatus::SUCCESS;
}

void MoveArmToDetectedObjectAction::on_failure(
  const typename rclcpp_action::ClientGoalHandle<MoveToObjectAction>::WrappedResult& result)
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
      return std::make_unique<sh_behavior_tree::MoveArmToDetectedObjectAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::MoveArmToDetectedObjectAction>(
    "MoveArmToDetectedObjectAction", builder);
}