#include "sh_behavior_tree/plugins/action/pick_and_place/clear_planning_scene_service.hpp"

namespace sh_behavior_tree
{

ClearPlanningSceneService::ClearPlanningSceneService(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_base_template::BTServiceNode<
      rclcpp_lifecycle::LifecycleNode, ClearPlanningSceneSrv>(action_name, node_config)
{}

ClearPlanningSceneService::~ClearPlanningSceneService()
{}

BT::PortsList ClearPlanningSceneService::providedPorts()
{
  return providedBasicPorts({});
}

bool ClearPlanningSceneService::send_request(std::shared_ptr<ClearPlanningSceneSrv::Request>& request)
{
  (void) request;
  // ClearPlanningSceneSrv::Request is empty
  return true;
}

BT::NodeStatus ClearPlanningSceneService::onTick(
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
      return std::make_unique<sh_behavior_tree::ClearPlanningSceneService>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::ClearPlanningSceneService>(
    "ClearPlanningSceneService", builder);
}