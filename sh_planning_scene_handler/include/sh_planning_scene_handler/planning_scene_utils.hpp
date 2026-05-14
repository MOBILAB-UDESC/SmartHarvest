#ifndef SH_PLANNING_SCENE_HANDLER__PLANNING_SCENE_UTILS_HPP_
#define SH_PLANNING_SCENE_HANDLER__PLANNING_SCENE_UTILS_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"

namespace sh_planning_scene_handler
{

uint8_t get_primitive(const std::string& shape);

void add_single_object_to_the_scene(
  moveit_msgs::msg::CollisionObject& collision_object,
  const std::string& object_name,
  const std::string& object_frame_id,
  const uint8_t& object_shape,
  const std::vector<double>& object_size,
  const geometry_msgs::msg::Pose& pose);

}  // namespace sh_planning_scene_handler

#endif  // SH_PLANNING_SCENE_HANDLER__PLANNING_SCENE_UTILS_HPP_