#include "sh_behavior_tree/plugins/action/detection/predict_action.hpp"

namespace sh_behavior_tree
{

PredictAction::PredictAction(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_bt_base_template::BTServiceNode<
      rclcpp_lifecycle::LifecycleNode, DetectObjectsSrv>(action_name, node_config)
{}

PredictAction::~PredictAction()
{}

BT::PortsList PredictAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<ImageMsg::ConstSharedPtr>("rgb_image"),
    BT::InputPort<ImageMsg::ConstSharedPtr>("depth_image"),
    BT::InputPort<CameraInfoMsg::ConstSharedPtr>("cam_info"),
    BT::OutputPort<sh_interfaces::msg::DetectedObjects>("objects", "List of detected objects")
  });
}

bool PredictAction::send_request(std::shared_ptr<DetectObjectsSrv::Request>& request)
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

  request->rgb_image = *rgb_image;
  request->depth_image = *depth_image;
  request->cam_info = *cam_info;

  return true;
}

BT::NodeStatus PredictAction::onTick(
  const std::shared_ptr<DetectObjectsSrv::Response>& response)
{
  if (!response->success) {
    RCLCPP_WARN(logger_, "Might be a problem with the server.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("objects", response->detected_objects);
  RCLCPP_INFO(logger_, "Response messages received");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::PredictAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::PredictAction>("PredictAction", builder);
}