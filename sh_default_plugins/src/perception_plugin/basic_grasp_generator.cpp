#include "sh_default_plugins/perception_plugin/basic_grasp_generator.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sh_default_plugins
{

bool BasicGraspGenerator::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
{
  auto shared_node = node.lock();

  if (!shared_node->has_parameter("grasp.camera_to_end_effector_transform")) {
    shared_node->declare_parameter<std::vector<double>>(
      "grasp.camera_to_end_effector_transform", std::vector<double>{0.0, 0.0, 0.0});
  }
  shared_node->get_parameter(
    "grasp.camera_to_end_effector_transform",
    camera_to_end_effector_transform_);

  if (!shared_node->has_parameter("grasp.orientation_euler")) {
    shared_node->declare_parameter<std::vector<double>>(
      "grasp.orientation_euler", std::vector<double>{0.0, 0.0, 0.0});
  }
  shared_node->get_parameter(
    "grasp.orientation_euler",
    orientation_euler_);

  shared_node->get_parameter("use_sim_time", use_sim_time_);

  return true;
}

void BasicGraspGenerator::cleanup()
{
}

bool BasicGraspGenerator::generate_grasp(
  std::vector<geometry_msgs::msg::Pose>& poses)
{
  double roll = orientation_euler_[0] + camera_to_end_effector_transform_[0];
  double pitch = orientation_euler_[1] + camera_to_end_effector_transform_[1];
  double yaw = orientation_euler_[2] + camera_to_end_effector_transform_[2];

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  auto q_pose = tf2::toMsg(q);

  for (auto& pose: poses) {
    pose.orientation = q_pose;
  }

  return true;
}

}  // namespace sh_default_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_default_plugins::BasicGraspGenerator, sh_base_template::GraspGeneratorBase)