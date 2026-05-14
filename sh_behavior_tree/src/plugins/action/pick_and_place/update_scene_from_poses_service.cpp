#include "sh_behavior_tree/plugins/action/pick_and_place/update_scene_from_poses_service.hpp"

#include "sh_interfaces/msg/perception_scene.hpp"

namespace sh_behavior_tree
{

UpdateSceneFromPosesService::UpdateSceneFromPosesService(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_base_template::BTServiceNode<
      rclcpp_lifecycle::LifecycleNode, UpdatePlanningSceneFromPosesSrv>(action_name, node_config)
{}

UpdateSceneFromPosesService::~UpdateSceneFromPosesService()
{}

BT::PortsList UpdateSceneFromPosesService::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<sh_interfaces::msg::PerceptionScene>(
      "perception_scene",
      "Complete perception result containing detected objects, header, and processing metadata")
  });
}

bool UpdateSceneFromPosesService::send_request(
  std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Request>& request)
{
  if (!getInput("perception_scene", request->perception_scene)) {
    RCLCPP_ERROR(logger_, "Missing input 'perception_scene'.");
    return false;
  }

  RCLCPP_INFO(logger_, "Sending request with %ld objects.", request->perception_scene.objects.size());
  return true;
}

BT::NodeStatus UpdateSceneFromPosesService::onTick(
  const std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Response>& response)
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
      return std::make_unique<sh_behavior_tree::UpdateSceneFromPosesService>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::UpdateSceneFromPosesService>(
    "UpdateSceneFromPosesService", builder);
}