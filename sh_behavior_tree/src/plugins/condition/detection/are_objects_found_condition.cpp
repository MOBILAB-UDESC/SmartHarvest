#include "sh_behavior_tree/plugins/condition/detection/are_objects_found_condition.hpp"

#include "sh_interfaces/msg/perception_scene.hpp"

namespace sh_behavior_tree
{

AreObjectsFoundCondition::AreObjectsFoundCondition(
  const std::string& condition_name, const BT::NodeConfig& config):
  BT::ConditionNode(condition_name, config), logger_(rclcpp::get_logger(condition_name))
{
  RCLCPP_INFO(logger_, "Node created.");
}

BT::PortsList AreObjectsFoundCondition::providedPorts()
{
  return {
    BT::InputPort<sh_interfaces::msg::PerceptionScene>(
      "perception_scene",
      "Complete perception result containing detected objects, header, and processing metadata"),
    BT::OutputPort<int>("total_objects", "Number of detected objects")
  };
}

BT::NodeStatus AreObjectsFoundCondition::tick()
{
  RCLCPP_INFO(logger_, "Checking if any object was found.");
  sh_interfaces::msg::PerceptionScene perception_scene;

  if (!getInput("perception_scene", perception_scene)) {
    RCLCPP_WARN(logger_, "Missing input 'perception_scene'.");
    return BT::NodeStatus::FAILURE;
  }

  int n_objects = perception_scene.objects.size();
  if (!n_objects) {
    RCLCPP_WARN(logger_, "No objects detected.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput<int>("total_objects", n_objects);
  RCLCPP_INFO(logger_, "%d objects detected.", n_objects);
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