#include "sh_behavior_tree/plugins/action/pick_and_place/get_next_object_to_pick.hpp"


namespace sh_behavior_tree
{

GetNextObjectToPick::GetNextObjectToPick(
  const std::string& action_name, const BT::NodeConfig& config):
  BT::SyncActionNode(action_name, config), logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Node created.");
}

BT::PortsList GetNextObjectToPick::providedPorts()
{
  return{
    BT::InputPort<int>("in_current_count", "Current count to pick."),
    BT::InputPort<std::string>("object_name_prefix", "Object name prefix."),
    BT::OutputPort<std::string>("current_object_name", "Object name.")
  };
}

BT::NodeStatus GetNextObjectToPick::tick()
{
  int in_current_count;
  std::string object_prefix;
  getInput("in_current_count", in_current_count);
  getInput("object_prefix", object_prefix);

  setOutput("current_object_name", object_prefix+"_"+std::to_string(in_current_count));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::GetNextObjectToPick>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::GetNextObjectToPick>(
    "GetNextObjectToPick", builder);
}