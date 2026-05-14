#include "sh_rgbd_perception_pipeline/default_grasp_generator.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sh_utils/ros2_node_utils.hpp"

namespace sh_rgbd_perception_pipeline
{

bool DefaultGraspGenerator::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
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

  shared_node->get_parameter("use_sim_time", use_sim_time_);

  return true;
}

void DefaultGraspGenerator::cleanup()
{
}

bool DefaultGraspGenerator::generate_grasp(
  const sh_base_template::types::PerceptionFrame& /*frame*/,
  std::vector<sh_base_template::types::PoseFeatures>& poses)
{
  for (auto& pose: poses) {
    if (!pose.valid_pose) {
      continue;
    }

    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double z = pose.pose.position.z;

    double roll, pitch, yaw;

    if (use_sim_time_) {
      roll  = std::atan2(z, x);
      yaw   = std::atan2(y, x);
      pitch = 0.0;
    }
    else {
      roll  = std::atan2(x, z);
      yaw   = 0.0;
      pitch = std::atan2(-y, std::sqrt(x*x + z*z));
    }

    roll += camera_to_end_effector_transform_[0];
    pitch += camera_to_end_effector_transform_[1];
    yaw += camera_to_end_effector_transform_[2];

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    pose.pose.orientation = tf2::toMsg(q);
  }

  return true;
}

}  // namespace sh_rgbd_perception_pipeline

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_rgbd_perception_pipeline::DefaultGraspGenerator, sh_base_template::GraspGeneratorBase)