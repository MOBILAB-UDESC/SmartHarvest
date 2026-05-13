#include "sh_behavior_tree/plugins/action/detection/predict_service.hpp"

#include "sh_interfaces/msg/perception_scene.hpp"

namespace sh_behavior_tree
{

PredictService::PredictService(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_base_template::BTServiceNode<
      rclcpp_lifecycle::LifecycleNode, RunPerceptionSrv>(action_name, node_config)
{}

PredictService::~PredictService()
{}

BT::PortsList PredictService::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<ImageMsg::ConstSharedPtr>("rgb_image"),
    BT::InputPort<ImageMsg::ConstSharedPtr>("depth_image"),
    BT::InputPort<CameraInfoMsg::ConstSharedPtr>("cam_info"),
    BT::OutputPort<sh_interfaces::msg::PerceptionScene>(
      "perception_scene",
      "Complete perception result containing detected objects, header, and processing metadata")
  });
}

bool PredictService::send_request(std::shared_ptr<RunPerceptionSrv::Request>& request)
{
  ImageMsg::ConstSharedPtr rgb_image;
  ImageMsg::ConstSharedPtr depth_image;
  CameraInfoMsg::ConstSharedPtr cam_info;

  if (!getInput("rgb_image", rgb_image) || !rgb_image) {
    RCLCPP_ERROR(logger_, "Missing or null input 'rgb_image'.");
    return false;
  }
  if (!getInput("depth_image", depth_image) || !depth_image) {
    RCLCPP_ERROR(logger_, "Missing or null input 'depth_image'.");
    return false;
  }
  if (!getInput("cam_info", cam_info) || !cam_info) {
    RCLCPP_ERROR(logger_, "Missing or null input 'cam_info'.");
    return false;
  }

  RCLCPP_INFO(logger_, "Sending a request.");

  request->input.rgb_images.resize(1);
  request->input.depth_images.resize(1);
  request->input.camera_infos.resize(1);

  request->input.rgb_images[0] = *rgb_image;
  request->input.depth_images[0] = *depth_image;
  request->input.camera_infos[0] = *cam_info;

  return true;
}

BT::NodeStatus PredictService::onTick(
  const std::shared_ptr<RunPerceptionSrv::Response>& response)
{
  if (!response->success) {
    RCLCPP_WARN(logger_, "Might be a problem with the server.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("perception_scene", response->scene);

  RCLCPP_INFO(logger_, "Response messages received in %.3f", response->scene.processing_time_ms);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::PredictService>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::PredictService>("PredictService", builder);
}