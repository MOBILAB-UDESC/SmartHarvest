#include "sh_basic_moveit_pipeline/basic_orientation_constraint.hpp"

namespace sh_basic_moveit_pipeline
{

bool BasicOrientationConstraint::apply_constraints(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface,
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools>& moveit_visual_tools,
  const geometry_msgs::msg::Pose& goal_pose)
{
  auto current_pose = move_group_interface->getCurrentPose(move_group_interface->getEndEffectorLink());

  moveit_msgs::msg::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = move_group_interface->getPoseReferenceFrame();
  orientation_constraint.link_name = move_group_interface->getEndEffectorLink();

  orientation_constraint.orientation = current_pose.pose.orientation;
  orientation_constraint.absolute_x_axis_tolerance = 0.4;
  orientation_constraint.absolute_y_axis_tolerance = 0.4;
  orientation_constraint.absolute_z_axis_tolerance = 0.4;
  orientation_constraint.weight = 1.0;

  moveit_msgs::msg::Constraints constraints;
  constraints.orientation_constraints.emplace_back(orientation_constraint);

  geometry_msgs::msg::Pose update_pose = goal_pose;
  update_pose.orientation = orientation_constraint.orientation;

  move_group_interface->setPathConstraints(constraints);
  move_group_interface->setPoseTarget(update_pose);

  return true;
}

}  // namespace sh_basic_moveit_pipeline

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_basic_moveit_pipeline::BasicOrientationConstraint, sh_base_template::MotionConstraintGeneratorBase)