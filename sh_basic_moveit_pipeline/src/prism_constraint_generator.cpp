#include "sh_basic_moveit_pipeline/prism_constraint_generator.hpp"

namespace sh_basic_moveit_pipeline
{

bool PrismConstraintGenerator::apply_constraints(
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface,
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools>& moveit_visual_tools,
  const geometry_msgs::msg::Pose& goal_pose)
{
  auto current_pose = move_group_interface->getCurrentPose(move_group_interface->getEndEffectorLink());
  moveit_visual_tools->publishSphere(current_pose.pose, rviz_visual_tools::RED, 0.05);
  moveit_visual_tools->publishSphere(goal_pose, rviz_visual_tools::GREEN, 0.05);

  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = move_group_interface->getPoseReferenceFrame();
  box_constraint.link_name = move_group_interface->getEndEffectorLink();

  shape_msgs::msg::SolidPrimitive box;

  // Center BOX
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.1, 0.1, 0.1 };
  box.dimensions[0] += 2*std::abs(goal_pose.position.x - current_pose.pose.position.x);
  box.dimensions[1] += 2*std::abs(goal_pose.position.y - current_pose.pose.position.y);
  box.dimensions[2] += 2*std::abs(goal_pose.position.z - current_pose.pose.position.z);

  box_constraint.constraint_region.primitives.emplace_back(box);

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = current_pose.pose.position.x;
  box_pose.position.y = current_pose.pose.position.y;
  box_pose.position.z = current_pose.pose.position.z;
  box_pose.orientation.w = 1.0;

  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;

  Eigen::Vector3d new_box_point_1(box_pose.position.x - box.dimensions[0] / 2,
                                  box_pose.position.y - box.dimensions[1] / 2,
                                  box_pose.position.z - box.dimensions[2] / 2);
  Eigen::Vector3d new_box_point_2(box_pose.position.x + box.dimensions[0] / 2,
                                  box_pose.position.y + box.dimensions[1] / 2,
                                  box_pose.position.z + box.dimensions[2] / 2);

  moveit_visual_tools->publishCuboid(new_box_point_1, new_box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
  moveit_visual_tools->trigger();

  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);

  move_group_interface->setPathConstraints(box_constraints);

  // moveit_msgs::msg::PositionConstraint plane_constraint;
  // plane_constraint.header.frame_id = move_group_interface->getPoseReferenceFrame();
  // plane_constraint.link_name = move_group_interface->getEndEffectorLink();
  // shape_msgs::msg::SolidPrimitive plane;
  // plane.type = shape_msgs::msg::SolidPrimitive::BOX;
  // plane.dimensions = { 0.1, 0.5, 0.1 };
  // plane.dimensions[0] += 2*std::abs(goal_pose.position.x - current_pose.pose.position.x);
  // // plane.dimensions[1] += 2*std::abs(goal_pose.position.y - current_pose.pose.position.y);
  // plane.dimensions[2] += 2*std::abs(goal_pose.position.z - current_pose.pose.position.z);
  // plane_constraint.constraint_region.primitives.emplace_back(plane);

  // geometry_msgs::msg::Pose plane_pose;
  // plane_pose.position.x = (current_pose.pose.position.x + goal_pose.position.x) / 2;
  // plane_pose.position.y = (current_pose.pose.position.y + goal_pose.position.y) / 2;
  // plane_pose.position.z = (current_pose.pose.position.z + goal_pose.position.z) / 2;
  // plane_pose.orientation.x = sin(M_PI_4 / 2);
  // plane_pose.orientation.y = 0.0;
  // plane_pose.orientation.z = 0.0;
  // plane_pose.orientation.w = cos(M_PI_4 / 2);
  // plane_constraint.constraint_region.primitive_poses.emplace_back(plane_pose);
  // plane_constraint.weight = 1.0;

  // // Visualize the constraint
  // auto d = sqrt(pow(goal_pose.position.y, 2) + pow(goal_pose.position.z, 2));

  // Eigen::Vector3d normal(0, 1, 1);
  // moveit_visual_tools->publishNormalAndDistancePlane(normal, d, rviz_visual_tools::TRANSLUCENT_DARK);
  // moveit_visual_tools->trigger();

  // moveit_msgs::msg::Constraints plane_constraints;
  // plane_constraints.position_constraints.emplace_back(plane_constraint);
  // plane_constraints.name = "use_equality_constraints";
  // move_group_interface->setPathConstraints(plane_constraints);
  return true;
}

}  // namespace sh_basic_moveit_pipeline

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sh_basic_moveit_pipeline::PrismConstraintGenerator, sh_base_template::MotionConstraintGeneratorBase)