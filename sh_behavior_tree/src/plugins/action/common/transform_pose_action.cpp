#include "sh_behavior_tree/plugins/action/common/transform_pose_action.hpp"

namespace sh_behavior_tree
{

TransformPoseAction::TransformPoseAction(
  const std::string& action_name, const BT::NodeConfig& config):
  BT::SyncActionNode(action_name, config), logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Initializing the TF listener.");

  node_ = config.blackboard->get<rclcpp_lifecycle::LifecycleNode::WeakPtr>("root_node");
  camera_to_end_effector_transform_ = std::vector<double>({-1.571, 0.0, -1.571});

  auto node = node_.lock();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(logger_, "Node created.");
}

BT::NodeStatus TransformPoseAction::tick()
{
  getInput("objects", objects_);

  if (!transform_pose()) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

bool TransformPoseAction::transform_pose()
{
  std::string transform_frame;
  getInput("transform_frame", transform_frame);
  int transformation_timeout = 1;

  sh_interfaces::msg::DetectedObjects transformed_objects;
  transformed_objects.objects.reserve(objects_.num_objects);
  transformed_objects.num_objects = objects_.num_objects;
  transformed_objects.header.frame_id = transform_frame;
  {
    auto node = node_.lock();
    transformed_objects.header.stamp = node->now();
  }

  geometry_msgs::msg::PoseStamped pose_in_stamped;
  pose_in_stamped.header = objects_.header;
  geometry_msgs::msg::PoseStamped pose_out_stamped;
  sh_interfaces::msg::SingleObjectInfo transformed_object_info;
  tf2::Quaternion orientation;

  for (auto obj : objects_.objects)
  {
    // Combine the static camera-to-end-effector Euler offset with the
    // object's grasp orientation to compute the final orientation in the source frame
    double roll = camera_to_end_effector_transform_[0] + obj.grasp_orientation_euler[0];
    double pitch = camera_to_end_effector_transform_[1] + obj.grasp_orientation_euler[1];
    double yaw = camera_to_end_effector_transform_[2] + obj.grasp_orientation_euler[2];
    orientation.setRPY(roll, pitch, yaw);

    pose_in_stamped.pose.position.x = obj.x;
    pose_in_stamped.pose.position.y = obj.y;
    pose_in_stamped.pose.position.z = obj.z;
    pose_in_stamped.pose.orientation = tf2::toMsg(orientation);

    // Transformation into transform_frame
    try {
      pose_out_stamped = tf_buffer_->transform(
        pose_in_stamped, transform_frame, tf2::durationFromSec(transformation_timeout));
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

    transformed_object_info.grasp_orientation_euler[0] = roll;
    transformed_object_info.grasp_orientation_euler[1] = pitch;
    transformed_object_info.grasp_orientation_euler[2] = yaw;

    // RCLCPP_INFO(node_->get_logger(), "Tranformed Pose x: %f", pose_out_stamped.pose.position.x);
    // RCLCPP_INFO(node_->get_logger(), "Tranformed Pose y: %f", pose_out_stamped.pose.position.y);
    // RCLCPP_INFO(node_->get_logger(), "Tranformed Pose z: %f", pose_out_stamped.pose.position.z);
    // RCLCPP_INFO(node_->get_logger(), "Tranformed Quat x: %f", pose_out_stamped.pose.orientation.x);
    // RCLCPP_INFO(node_->get_logger(), "Tranformed Quat y: %f", pose_out_stamped.pose.orientation.y);
    // RCLCPP_INFO(node_->get_logger(), "Tranformed Quat z: %f", pose_out_stamped.pose.orientation.z);
    // RCLCPP_INFO(node_->get_logger(), "Tranformed Quat w: %f", pose_out_stamped.pose.orientation.w);

    transformed_objects.objects.push_back(transformed_object_info);
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
      return std::make_unique<sh_behavior_tree::TransformPoseAction>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::TransformPoseAction>(
    "TransformPoseAction", builder);
}