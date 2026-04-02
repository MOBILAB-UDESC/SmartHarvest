#include "sh_behavior_tree/plugins/action/pick_and_place/compute_grasp_orientation.hpp"

namespace sh_behavior_tree
{

ComputeGraspOrientation::ComputeGraspOrientation(
  const std::string& action_name, const BT::NodeConfig& config) :
  BT::SyncActionNode(action_name, config),
  logger_(rclcpp::get_logger(action_name))
{
  camera_to_end_effector_transform_ =
    config.blackboard->get<std::vector<double>>("camera_to_end_effector_transform");

  if (camera_to_end_effector_transform_.size() != 3) {
    throw std::invalid_argument(
      "camera_to_end_effector_transform must have 3 values (roll, pitch, yaw).");
  }

  RCLCPP_INFO(logger_, "Node created.");
}

BT::NodeStatus ComputeGraspOrientation::tick()
{
  sh_interfaces::msg::DetectedObjects objects;
  if (!getInput("objects", objects)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'objects'.");
    return BT::NodeStatus::FAILURE;
  }
  tf2::Quaternion orientation;
  geometry_msgs::msg::Quaternion quaternion;
  int index = 1;

  for (auto& object: objects.objects) {
    object.grasp_orientation_euler[0] += camera_to_end_effector_transform_[0];
    object.grasp_orientation_euler[1] += camera_to_end_effector_transform_[1];
    object.grasp_orientation_euler[2] += camera_to_end_effector_transform_[2];
    orientation.setRPY(
      object.grasp_orientation_euler[0],
      object.grasp_orientation_euler[1],
      object.grasp_orientation_euler[2]);

    quaternion = tf2::toMsg(orientation);

    object.grasp_orientation_quaternion[0] = quaternion.x;
    object.grasp_orientation_quaternion[1] = quaternion.y;
    object.grasp_orientation_quaternion[2] = quaternion.z;
    object.grasp_orientation_quaternion[3] = quaternion.w;

    RCLCPP_DEBUG(logger_, "Object %d grasp orientation:", index);
    RCLCPP_DEBUG(logger_, "  Grasp roll: %.3f", object.grasp_orientation_euler[0]);
    RCLCPP_DEBUG(logger_, "  Grasp pitch: %.3f", object.grasp_orientation_euler[1]);
    RCLCPP_DEBUG(logger_, "  Grasp yaw: %.3f", object.grasp_orientation_euler[2]);
    RCLCPP_DEBUG(logger_, "  Grasp Quat x: %.3f", object.grasp_orientation_quaternion[0]);
    RCLCPP_DEBUG(logger_, "  Grasp Quat y: %.3f", object.grasp_orientation_quaternion[1]);
    RCLCPP_DEBUG(logger_, "  Grasp Quat z: %.3f", object.grasp_orientation_quaternion[2]);
    RCLCPP_DEBUG(logger_, "  Grasp Quat w: %.3f", object.grasp_orientation_quaternion[3]);

    ++index;
  }

  setOutput("objects", objects);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::ComputeGraspOrientation>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::ComputeGraspOrientation>(
    "ComputeGraspOrientation", builder);
}