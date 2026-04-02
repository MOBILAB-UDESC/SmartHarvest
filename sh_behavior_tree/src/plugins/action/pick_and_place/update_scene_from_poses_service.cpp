#include "sh_behavior_tree/plugins/action/pick_and_place/update_scene_from_poses_service.hpp"

namespace sh_behavior_tree
{

UpdateSceneFromPosesService::UpdateSceneFromPosesService(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_bt_base_template::BTServiceNode<
      rclcpp_lifecycle::LifecycleNode, UpdatePlanningSceneFromPosesSrv>(action_name, node_config)
{}

UpdateSceneFromPosesService::~UpdateSceneFromPosesService()
{}

BT::PortsList UpdateSceneFromPosesService::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<sh_interfaces::msg::DetectedObjects>("objects", "List of detected objects")
  });
}

bool UpdateSceneFromPosesService::send_request(
  std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Request>& request)
{
  if (!getInput("objects", request->detected_objects)) {
    RCLCPP_ERROR(logger_, "Missing input 'detected_objects'.");
    return false;
  }

  RCLCPP_INFO(logger_, "Sending request with %d objects.", request->detected_objects.num_objects);
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