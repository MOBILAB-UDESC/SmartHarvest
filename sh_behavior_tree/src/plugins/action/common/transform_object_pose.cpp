#include "sh_behavior_tree/plugins/action/common/transform_object_pose.hpp"

namespace sh_behavior_tree
{

TransformObjectPose::TransformObjectPose(
  const std::string& action_name, const BT::NodeConfig& config):
  BT::SyncActionNode(action_name, config), logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_DEBUG(logger_, "Initializing the TF listener.");

  node_ = config.blackboard->get<rclcpp_lifecycle::LifecycleNode::WeakPtr>("root_node");
  transformation_timeout_ = config.blackboard->get<double>("transformation_timeout");

  auto node = node_.lock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_DEBUG(logger_, "Node created.");
}

BT::NodeStatus TransformObjectPose::tick()
{
  sh_interfaces::msg::DetectedObjects objects;
  std::string transform_frame;
  if (!getInput("objects", objects)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'objects'.");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("transform_frame", transform_frame)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'transform_frame'.");
    return BT::NodeStatus::FAILURE;
  }

  if (!transform_pose(objects, transform_frame)) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

bool TransformObjectPose::transform_pose(
  const sh_interfaces::msg::DetectedObjects& objects,
  const std::string& transform_frame)
{
  // Lis of transformed objects
  sh_interfaces::msg::DetectedObjects transformed_objects;
  transformed_objects.objects.reserve(objects.num_objects);
  transformed_objects.num_objects = objects.num_objects;
  transformed_objects.header.frame_id = transform_frame;
  {
    auto node = node_.lock();
    transformed_objects.header.stamp = node->now();
  }

  // Transformation handlers
  geometry_msgs::msg::PoseStamped pose_in_stamped;
  pose_in_stamped.header = objects.header;
  geometry_msgs::msg::PoseStamped pose_out_stamped;
  sh_interfaces::msg::SingleObjectInfo transformed_object_info;

  tf2::Quaternion q;
  int index = 1;

  for (auto obj : objects.objects)
  {
    pose_in_stamped.pose.position.x = obj.x;
    pose_in_stamped.pose.position.y = obj.y;
    pose_in_stamped.pose.position.z = obj.z;
    pose_in_stamped.pose.orientation.x = obj.grasp_orientation_quaternion[0];
    pose_in_stamped.pose.orientation.y = obj.grasp_orientation_quaternion[1];
    pose_in_stamped.pose.orientation.z = obj.grasp_orientation_quaternion[2];
    pose_in_stamped.pose.orientation.w = obj.grasp_orientation_quaternion[3];

    // Transformation into transform_frame
    try {
      pose_out_stamped = tf_buffer_->transform(
        pose_in_stamped, transform_frame, tf2::durationFromSec(transformation_timeout_));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(logger_,
        "Could not transform object pose to %s: %s", transform_frame.c_str(), ex.what());
      return false;
    }

    transformed_object_info = obj;
    transformed_object_info.x = pose_out_stamped.pose.position.x;
    transformed_object_info.y = pose_out_stamped.pose.position.y;
    transformed_object_info.z = pose_out_stamped.pose.position.z;

    transformed_object_info.grasp_orientation_quaternion[0] = pose_out_stamped.pose.orientation.x;
    transformed_object_info.grasp_orientation_quaternion[1] = pose_out_stamped.pose.orientation.y;
    transformed_object_info.grasp_orientation_quaternion[2] = pose_out_stamped.pose.orientation.z;
    transformed_object_info.grasp_orientation_quaternion[3] = pose_out_stamped.pose.orientation.w;

    // Quaternion to Euler transformation
    tf2::fromMsg(pose_out_stamped.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(
      transformed_object_info.grasp_orientation_euler[0],
      transformed_object_info.grasp_orientation_euler[1],
      transformed_object_info.grasp_orientation_euler[2]);

    RCLCPP_DEBUG(logger_, "Object %d pose:", index);
    RCLCPP_DEBUG(logger_, "  Tranformed Pose x: %f", pose_out_stamped.pose.position.x);
    RCLCPP_DEBUG(logger_, "  Tranformed Pose y: %f", pose_out_stamped.pose.position.y);
    RCLCPP_DEBUG(logger_, "  Tranformed Pose z: %f", pose_out_stamped.pose.position.z);
    RCLCPP_DEBUG(logger_, "  Tranformed Quat x: %f", pose_out_stamped.pose.orientation.x);
    RCLCPP_DEBUG(logger_, "  Tranformed Quat y: %f", pose_out_stamped.pose.orientation.y);
    RCLCPP_DEBUG(logger_, "  Tranformed Quat z: %f", pose_out_stamped.pose.orientation.z);
    RCLCPP_DEBUG(logger_, "  Tranformed Quat w: %f", pose_out_stamped.pose.orientation.w);

    // Stack objects
    transformed_objects.objects.push_back(transformed_object_info);
    ++index;
  }

  setOutput("transformed_objects", transformed_objects);

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