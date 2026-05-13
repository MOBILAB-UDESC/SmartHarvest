#include "sh_behavior_tree/plugins/action/common/transform_object_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sh_behavior_tree
{

TransformObjectPose::TransformObjectPose(
  const std::string& action_name, const BT::NodeConfig& config):
  BT::SyncActionNode(action_name, config), logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Initializing the TF listener.");

  node_ = config.blackboard->get<rclcpp_lifecycle::LifecycleNode::WeakPtr>("root_node");
  transformation_timeout_ = config.blackboard->get<double>("transformation_timeout");

  auto node = node_.lock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(logger_, "Node created.");
}

BT::NodeStatus TransformObjectPose::tick()
{
  sh_interfaces::msg::PerceptionScene perception_scene;
  std::string transform_frame;
  if (!getInput("perception_scene", perception_scene)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'perception_scene'.");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("transformation_frame", transform_frame)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'transformation_frame'.");
    return BT::NodeStatus::FAILURE;
  }

  if (!transform_pose(perception_scene, transform_frame)) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

bool TransformObjectPose::transform_pose(
  const sh_interfaces::msg::PerceptionScene& perception_scene,
  const std::string& transform_frame)
{
  int num_objects = perception_scene.objects.size();
  // Lis of transformed objects
  sh_interfaces::msg::PerceptionScene transformed_perception_scene = perception_scene;
  transformed_perception_scene.header.frame_id = transform_frame;
  // {
  //   auto node = node_.lock();
  //   transformed_perception_scene.header.stamp = node->now();
  // }

  // Transformation handlers
  geometry_msgs::msg::PoseStamped pose_in_stamped;
  pose_in_stamped.header = perception_scene.header;
  geometry_msgs::msg::PoseStamped pose_out_stamped;

  tf2::Quaternion q;

  for (int i = 0; i < num_objects; ++i) {
    pose_in_stamped.pose = perception_scene.objects[i].pose;

    // Transformation into transform_frame
    try {
      pose_out_stamped = tf_buffer_->transform(
        pose_in_stamped, transform_frame, tf2::durationFromSec(transformation_timeout_));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(logger_,
        "Could not transform object pose to %s: %s", transform_frame.c_str(), ex.what());
      return false;
    }

    RCLCPP_DEBUG(logger_, "Object %d pose:", i+1);
    RCLCPP_DEBUG(logger_, "  Tranformed Pose x: %f", pose_out_stamped.pose.position.x);
    RCLCPP_DEBUG(logger_, "  Tranformed Pose y: %f", pose_out_stamped.pose.position.y);
    RCLCPP_DEBUG(logger_, "  Tranformed Pose z: %f", pose_out_stamped.pose.position.z);
    RCLCPP_DEBUG(logger_, "  Tranformed Quat x: %f", pose_out_stamped.pose.orientation.x);
    RCLCPP_DEBUG(logger_, "  Tranformed Quat y: %f", pose_out_stamped.pose.orientation.y);
    RCLCPP_DEBUG(logger_, "  Tranformed Quat z: %f", pose_out_stamped.pose.orientation.z);
    RCLCPP_DEBUG(logger_, "  Tranformed Quat w: %f", pose_out_stamped.pose.orientation.w);

    // Stack objects
    transformed_perception_scene.objects[i].pose = pose_out_stamped.pose;
  }

  setOutput("transformed_perception_scene", transformed_perception_scene);
  RCLCPP_INFO(logger_, "Objects transformed.");

  return true;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::TransformObjectPose>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::TransformObjectPose>(
    "TransformObjectPose", builder);
}