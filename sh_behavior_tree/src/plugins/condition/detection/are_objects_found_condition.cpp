#include "sh_behavior_tree/plugins/condition/detection/are_objects_found_condition.hpp"

namespace sh_behavior_tree
{

AreObjectsFoundCondition::AreObjectsFoundCondition(
  const std::string& condition_name, const BT::NodeConfig& config):
  BT::ConditionNode(condition_name, config), logger_(rclcpp::get_logger(condition_name))
{
  RCLCPP_INFO(logger_, "Node created.");
}

BT::NodeStatus AreObjectsFoundCondition::tick()
{
  RCLCPP_INFO(logger_, "Checking if any object was found.");

  sh_interfaces::msg::DetectedObjects obj_pose;
  getInput("objects", obj_pose);
  setOutput("total_objects", obj_pose.num_objects);

  if (!obj_pose.num_objects) {
    RCLCPP_WARN(logger_, "No objects detected");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger_, "%d detected objects.", obj_pose.num_objects);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::AreObjectsFoundCondition>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::AreObjectsFoundCondition>(
    "AreObjectsFoundCondition", builder);
}