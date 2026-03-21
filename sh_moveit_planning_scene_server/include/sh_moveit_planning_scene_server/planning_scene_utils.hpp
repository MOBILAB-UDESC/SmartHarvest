#ifndef SH_MOVEIT_PLANNING_SCENE_SERVER__PLANNING_SCENE_UTILS_HPP_
#define SH_MOVEIT_PLANNING_SCENE_SERVER__PLANNING_SCENE_UTILS_HPP_

#include "moveit_msgs/msg/collision_object.hpp"

namespace sh_moveit_planning_scene_server
{

uint8_t get_primitive(const std::string& shape);

void add_single_object_to_the_scene(
  moveit_msgs::msg::CollisionObject& collision_object,
  const std::string& object_name,
  const std::string& object_frame_id,
  const uint8_t& object_shape,
  const std::vector<double>& object_size,
  const std::vector<double>& object_position,
  const std::vector<double>& object_grasp_orientation);

}  // namespace sh_moveit_planning_scene_server

#endif  // SH_MOVEIT_PLANNING_SCENE_SERVER__PLANNING_SCENE_UTILS_HPP_