#include "sh_behavior_tree/plugins/action/pick_and_place/check_error_ids.hpp"

namespace sh_behavior_tree
{

CheckErrorIds::CheckErrorIds(
  const std::string& action_name, const BT::NodeConfig& config):
  BT::SyncActionNode(action_name, config), logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Node created.");
}

BT::PortsList CheckErrorIds::providedPorts()
{
  return{
    BT::InputPort<int>("error_id", "Error ID."),
    BT::InputPort<int>("in_current_count", "Current count of objects available to pick."),
    BT::InputPort<int>("total_objects", "Total."),
    BT::OutputPort<int>("out_next_count", "Updated count after the action."),
    BT::BidirectionalPort<int>("collected_object", "Collected")
  };
}

BT::NodeStatus CheckErrorIds::tick()
{
  int in_current_count, total_objects, collected_object;
  int error_id;
  getInput("in_current_count", in_current_count);
  getInput("total_objects", total_objects);
  getInput("error_id", error_id);
  getInput("collected_object", collected_object);

  using sh_interfaces::msg::ErrorCodes;
  switch (error_id)
  {
    case ErrorCodes::SUCCESS:
      ++collected_object;
      ++in_current_count;
      if (in_current_count >= total_objects) {
        setOutput("collected_object", collected_object);
        RCLCPP_INFO(logger_, "%d/%d objects have been collected.", collected_object, total_objects);
        return BT::NodeStatus::SUCCESS;
      }
      setOutput("out_next_count", in_current_count);
      RCLCPP_INFO(logger_, "There is %d objects available to pick", total_objects-in_current_count);
      return BT::NodeStatus::FAILURE;

    case ErrorCodes::TIMEOUT:
      ++in_current_count;
      if (in_current_count > total_objects) {
        setOutput("collected_object", collected_object);
        RCLCPP_INFO(logger_, "%d/%d objects have been collected.", collected_object, total_objects);
        return BT::NodeStatus::SUCCESS;
      }
      setOutput("out_next_count", in_current_count);
      RCLCPP_ERROR(logger_, "Response have not arrived in time.");
      return BT::NodeStatus::FAILURE;

    case ErrorCodes::GOAL_CANCELED:
      ++in_current_count;
      if (in_current_count > total_objects) {
        setOutput("collected_object", collected_object);
        RCLCPP_INFO(logger_, "%d/%d objects have been collected.", collected_object, total_objects);
        return BT::NodeStatus::SUCCESS;
      }
      setOutput("out_next_count", in_current_count);
      RCLCPP_ERROR(logger_, "Action goal canceled.");
      return BT::NodeStatus::FAILURE;

    case ErrorCodes::GOAL_REJECTED:
      ++in_current_count;
      if (in_current_count > total_objects) {
        setOutput("collected_object", collected_object);
        RCLCPP_INFO(logger_, "%d/%d objects have been collected.", collected_object, total_objects);
        return BT::NodeStatus::SUCCESS;
      }
      setOutput("out_next_count", in_current_count);
      RCLCPP_ERROR(logger_, "Action goal rejected.");
      return BT::NodeStatus::FAILURE;

    case ErrorCodes::PLANNING_FAILED:
      ++in_current_count;
      if (in_current_count > total_objects) {
        setOutput("collected_object", collected_object);
        RCLCPP_INFO(logger_, "%d/%d objects have been collected.", collected_object, total_objects);
        return BT::NodeStatus::SUCCESS;
      }
      setOutput("out_next_count", in_current_count);
      RCLCPP_ERROR(logger_, "Planning failed.");
      return BT::NodeStatus::FAILURE;

    case ErrorCodes::EXECUTION_FAILED:
      ++in_current_count;
      if (in_current_count > total_objects) {
        setOutput("collected_object", collected_object);
        RCLCPP_INFO(logger_, "%d/%d objects have been collected.", collected_object, total_objects);
        return BT::NodeStatus::SUCCESS;
      }
      setOutput("out_next_count", in_current_count);
      RCLCPP_ERROR(logger_, "Execution failed.");
      return BT::NodeStatus::FAILURE;
  }
  // return BT::NodeStatus::SUCCESS;

}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::CheckErrorIds>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::CheckErrorIds>(
    "CheckErrorIds", builder);
}