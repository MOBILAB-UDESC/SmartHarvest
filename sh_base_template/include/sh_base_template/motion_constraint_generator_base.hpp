#ifndef SH_BASE_TEMPLATE__MOTION_CONSTRAINT_GENERATOR_BASE_HPP_
#define SH_BASE_TEMPLATE__MOTION_CONSTRAINT_GENERATOR_BASE_HPP_

#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace sh_base_template
{

class MotionConstraintGeneratorBase
{
public:
  /**
   * @brief Configure transition.
   */
  virtual bool configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& /*node*/)
  {
    return true;
  }

  /**
   * @brief Activate transition.
   */
  virtual void activate() {};

  /**
   * @brief Deactivate transition.
   */
  virtual void deactivate() {};

  /**
   * @brief Cleanup transition.
   */
  virtual void cleanup() {};

  /**
   * @brief Motion constraint implementation.
   *
   * @param move_group_interface.
   * @return bool true or false.
   */
  virtual bool apply_constraints(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface,
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools>& moveit_visual_tools,
    const geometry_msgs::msg::Pose& goal_pose) = 0;

  /**
   * @brief Clear constraints applied in apply_constraints method.
   *
   * @param move_group_interface.
   */
  virtual void clear_constraints(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface,
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools>& moveit_visual_tools)
  {
    move_group_interface->clearPathConstraints();
    moveit_visual_tools->deleteAllMarkers();
    moveit_visual_tools->trigger();
  };

};

}  // namespace sh_base_template

#endif  // SH_BASE_TEMPLATE__MOTION_CONSTRAINT_GENERATOR_BASE_HPP_