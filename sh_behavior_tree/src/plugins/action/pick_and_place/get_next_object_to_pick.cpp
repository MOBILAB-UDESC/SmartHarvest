#include "sh_behavior_tree/plugins/action/pick_and_place/get_next_object_to_pick.hpp"


namespace sh_behavior_tree
{

GetNextObjectToPick::GetNextObjectToPick(
  const std::string& action_name, const BT::NodeConfig& config) :
    logger_(rclcpp::get_logger(action_name)),
    sh_base_template::BTServiceNode<
      rclcpp_lifecycle::LifecycleNode, SelectNextTargetSrv>(action_name, config)
{
  RCLCPP_INFO(logger_, "Node created.");
}

BT::PortsList GetNextObjectToPick::providedPorts()
{
  return providedBasicPorts({
    BT::OutputPort<std::string>("next_object_name", "sad")
  });
}

bool GetNextObjectToPick::send_request(std::shared_ptr<SelectNextTargetSrv::Request>& request)
{
  (void) request;
  // ClearPlanningSceneSrv::Request is empty
  return true;
}

BT::NodeStatus GetNextObjectToPick::onTick(
  const std::shared_ptr<SelectNextTargetSrv::Response>& response)
{
  if (!response->success) {
    RCLCPP_ERROR(logger_, "No more objects to pick.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("next_object_name", response->target_name);
  RCLCPP_INFO(logger_, "%s selected.", response->target_name.c_str());
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