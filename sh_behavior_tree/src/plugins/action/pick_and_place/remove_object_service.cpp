#include "sh_behavior_tree/plugins/action/pick_and_place/remove_object_service.hpp"

namespace sh_behavior_tree
{

RemoveObjectService::RemoveObjectService(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_base_template::BTServiceNode<
      rclcpp_lifecycle::LifecycleNode, RemoveObjectFromSceneSrv>(action_name, node_config)
{}

RemoveObjectService::~RemoveObjectService()
{}

BT::PortsList RemoveObjectService::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("object_name", "Object in the planning scene.")});
}

bool RemoveObjectService::send_request(std::shared_ptr<RemoveObjectFromSceneSrv::Request>& request)
{
  if (!getInput("object_name", request->object_name)) {
    RCLCPP_ERROR(logger_, "Missing input 'object_name'.");
    return false;
  }
  return true;
}

BT::NodeStatus RemoveObjectService::onTick(
  const std::shared_ptr<RemoveObjectFromSceneSrv::Response>& response)
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
      return std::make_unique<sh_behavior_tree::RemoveObjectService>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::RemoveObjectService>(
    "RemoveObjectService", builder);
}