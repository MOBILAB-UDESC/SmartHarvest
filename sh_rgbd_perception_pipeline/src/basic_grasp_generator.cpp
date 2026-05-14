#include "sh_rgbd_perception_pipeline/basic_grasp_generator.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sh_utils/ros2_node_utils.hpp"

namespace sh_rgbd_perception_pipeline
{

bool BasicGraspGenerator::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
{
  auto shared_node = node.lock();

  sh_utils::ros2_node_utils::declare_and_get_parameter<std::vector<double>>(
    shared_node, "pipeline.grasp.camera_to_end_effector_transform",
    camera_to_end_effector_transform_, std::vector<double>{0.0, 0.0, 0.0});

  if (camera_to_end_effector_transform_.size() != 3) {
    RCLCPP_WARN(
      shared_node->get_logger(),
      "'camera_to_end_effector_transform' param in grasp must be a 3D vector. Using default zero vector.");
    camera_to_end_effector_transform_.clear();
    camera_to_end_effector_transform_ = std::vector<double>{0.0, 0.0, 0.0};
  }

  sh_utils::ros2_node_utils::declare_and_get_parameter<std::vector<double>>(
    shared_node, "pipeline.grasp.orientation_euler",
    orientation_euler_, std::vector<double>{0.0, 0.0, 0.0});

  if (orientation_euler_.size() != 3) {
    RCLCPP_WARN(
      shared_node->get_logger(),
      "'orientation_euler' param in grasp must be a 3D vector. Using default zero vector.");
    orientation_euler_.clear();
    orientation_euler_ = std::vector<double>{0.0, 0.0, 0.0};
  }

  shared_node->get_parameter("use_sim_time", use_sim_time_);

  return true;
}

void BasicGraspGenerator::cleanup()
{
}

bool BasicGraspGenerator::generate_grasp(
  const sh_base_template::types::PerceptionFrame& /*frame*/,
  std::vector<sh_base_template::types::PoseFeatures>& poses)
{
  double roll = orientation_euler_[0] + camera_to_end_effector_transform_[0];
  double pitch = orientation_euler_[1] + camera_to_end_effector_transform_[1];
  double yaw = orientation_euler_[2] + camera_to_end_effector_transform_[2];

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  auto q_pose = tf2::toMsg(q);

  for (auto& pose: poses) {
    if (!pose.valid_pose) {
      continue;
    }
    pose.pose.orientation = q_pose;
  }

  return true;
}

}  // namespace sh_rgbd_perception_pipeline

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_rgbd_perception_pipeline::BasicGraspGenerator, sh_base_template::GraspGeneratorBase)