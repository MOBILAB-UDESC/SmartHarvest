#include "sh_planning_scene_handler/planning_scene_utils.hpp"

namespace sh_planning_scene_handler
{

uint8_t get_primitive(const std::string& shape)
{
  if (shape == "SPHERE") {
    return shape_msgs::msg::SolidPrimitive::SPHERE;
  } else if (shape == "BOX") {
    return shape_msgs::msg::SolidPrimitive::BOX;
  } else if (shape == "CYLINDER") {
    return shape_msgs::msg::SolidPrimitive::CYLINDER;
  } else if (shape == "CONE") {
    return shape_msgs::msg::SolidPrimitive::CONE;
  } else {
    throw std::runtime_error(
      "Shape not valid. Must be one of these: [SPHERE, BOX, CYLINDER or CONE]");
  }
}

void add_single_object_to_the_scene(
  moveit_msgs::msg::CollisionObject& collision_object,
  const std::string& object_name,
  const std::string& object_frame_id,
  const uint8_t& object_shape,
  const std::vector<double>& object_size,
  const geometry_msgs::msg::Pose& pose)
{
  collision_object.id = object_name;
  collision_object.header.frame_id = object_frame_id;

  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = object_shape;
  collision_object.primitives[0].dimensions.resize(object_size.size());
  collision_object.primitives[0].dimensions.assign(object_size.begin(), object_size.end());

  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0] = pose;

  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
}

}  // namespace sh_planning_scene_handler
