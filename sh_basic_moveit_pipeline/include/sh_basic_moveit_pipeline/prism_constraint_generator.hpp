#ifndef SH_BASIC_MOVEIT_PIPELINE__PRISM_CONSTRAINT_GENERATOR_HPP_
#define SH_BASIC_MOVEIT_PIPELINE__PRISM_CONSTRAINT_GENERATOR_HPP_

#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/motion_constraint_generator_base.hpp"

namespace sh_basic_moveit_pipeline
{

class PrismConstraintGenerator : public sh_base_template::MotionConstraintGeneratorBase
{
public:
  /**
   * @brief Motion constraint implementation.
   *
   * @param move_group_interface.
   * @return bool true or false.
   */
  bool apply_constraints(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface,
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools>& moveit_visual_tools,
    const geometry_msgs::msg::Pose& goal_pose);
};

}  // namespace sh_basic_moveit_pipeline

#endif  // SH_BASIC_MOVEIT_PIPELINE__PRISM_CONSTRAINT_GENERATOR_HPP_