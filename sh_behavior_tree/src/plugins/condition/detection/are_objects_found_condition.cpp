#include "sh_behavior_tree/plugins/condition/detection/are_objects_found_condition.hpp"

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
    BT::InputPort<sh_interfaces::msg::DetectedObjects>("objects", "List of detected objects"),
    BT::OutputPort<int>("total_objects", "Number of detected objects")
  };
}

BT::NodeStatus AreObjectsFoundCondition::tick()
{
  RCLCPP_INFO(logger_, "Checking if any object was found.");

  getInput("objects", detected_objects);

  if (!detected_objects.num_objects) {
    RCLCPP_WARN(logger_, "No objects detected");
    return BT::NodeStatus::FAILURE;
  }

  setOutput<int>("total_objects", detected_objects.num_objects);
  // setOutput("total_objects", detected_objects.num_objects);
  RCLCPP_INFO(logger_, "%d detected objects.", detected_objects.num_objects);
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