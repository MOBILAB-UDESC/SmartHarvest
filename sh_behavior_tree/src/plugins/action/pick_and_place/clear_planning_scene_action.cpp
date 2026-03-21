#include "sh_behavior_tree/plugins/action/pick_and_place/clear_planning_scene_action.hpp"

namespace sh_behavior_tree
{

ClearPlanningSceneAction::ClearPlanningSceneAction(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_bt_base_template::BTServiceNode<
      rclcpp_lifecycle::LifecycleNode, ClearPlanningSceneSrv>(action_name, node_config)
{}

ClearPlanningSceneAction::~ClearPlanningSceneAction()
{}

BT::PortsList ClearPlanningSceneAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name", "Service name")
  };
}

bool ClearPlanningSceneAction::send_request(std::shared_ptr<ClearPlanningSceneSrv::Request>& request)
{
  (void) request;
  // ClearPlanningSceneSrv::Request is empty
  return true;
}

BT::NodeStatus ClearPlanningSceneAction::onTick(
  const std::shared_ptr<ClearPlanningSceneSrv::Response>& response)
{
  if (!response->success) {
    RCLCPP_ERROR(logger_, response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger_, response->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::ClearPlanningSceneAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::ClearPlanningSceneAction>(
    "ClearPlanningSceneAction", builder);
}