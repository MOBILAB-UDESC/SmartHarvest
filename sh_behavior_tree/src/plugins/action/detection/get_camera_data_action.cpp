#include "sh_behavior_tree/plugins/action/detection/get_camera_data_action.hpp"

namespace sh_behavior_tree
{

GetCameraDataAction::GetCameraDataAction(
  const std::string & action_name,
  const BT::NodeConfig & node_config) :
    sh_bt_base_template::BTSyncSubscriptionNode<
      rclcpp_lifecycle::LifecycleNode, ImageMsg, ImageMsg, CameraInfoMsg>(action_name, node_config)
{}

GetCameraDataAction::~GetCameraDataAction()
{}

BT::PortsList GetCameraDataAction::providedPorts()
{
  return providedBasicPorts({
    BT::OutputPort<ImageMsg::ConstSharedPtr>("rgb_image"),
    BT::OutputPort<ImageMsg::ConstSharedPtr>("depth_image"),
    BT::OutputPort<CameraInfoMsg::ConstSharedPtr>("cam_info")
  });
}

BT::NodeStatus GetCameraDataAction::onTick(
  const std::shared_ptr<const ImageMsg>& rgb_msg_,
  const std::shared_ptr<const ImageMsg>& depth_msg_,
  const std::shared_ptr<const CameraInfoMsg>& cam_info_msg)
{
  setOutput("rgb_image", rgb_msg_);
  setOutput("depth_image", depth_msg_);
  setOutput("cam_info", cam_info_msg);

  RCLCPP_DEBUG(
    logger_, "Images of size %dx%d and %dx%d received.",
    rgb_msg_->width, rgb_msg_->height,
    depth_msg_->width, depth_msg_->height);

  RCLCPP_DEBUG(logger_, "RGB optical frame: %s", rgb_msg_->header.frame_id.c_str());
  RCLCPP_DEBUG(logger_, "Depth optical frame: %s", depth_msg_->header.frame_id.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::GetCameraDataAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::GetCameraDataAction>("GetCameraDataAction", builder);
}