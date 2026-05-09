#include "sh_default_plugins/perception_plugin/default_grasp_generator.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sh_default_plugins
{

bool DefaultGraspGenerator::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node)
{
  auto shared_node = node.lock();

  if (!shared_node->has_parameter("grasp.camera_to_end_effector_transform")) {
    shared_node->declare_parameter<std::vector<double>>(
      "grasp.camera_to_end_effector_transform", std::vector<double>{0.0, 0.0, 0.0});
  }
  shared_node->get_parameter(
    "grasp.camera_to_end_effector_transform",
    camera_to_end_effector_transform_);

  shared_node->get_parameter("use_sim_time", use_sim_time_);

  return true;
}

void DefaultGraspGenerator::cleanup()
{
}

bool DefaultGraspGenerator::generate_grasp(
  std::vector<geometry_msgs::msg::Pose>& poses)
{
  for (auto& pose: poses) {
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;

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

    pose.orientation = tf2::toMsg(q);
  }

  return true;
}

}  // namespace sh_default_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_default_plugins::DefaultGraspGenerator, sh_base_template::GraspGeneratorBase)