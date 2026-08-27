#include "sh_planning_scene_handler/planning_scene_handler.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"

#include "sh_planning_scene_handler/planning_scene_utils.hpp"

namespace sh_planning_scene_handler
{

PlanningSceneHandler::PlanningSceneHandler(const std::string& server_name) :
  rclcpp::Node(server_name),
  logger_(rclcpp::get_logger(server_name))
{
  RCLCPP_INFO(logger_, "Creating.");

  declare_parameters();
  target_primitive_ = get_primitive(get_parameter("target_shape").as_string());

  clear_service_ = this->create_service<ClearPlanningSceneSrv>(
    "clear_scene",
    std::bind(
      &PlanningSceneHandler::clear_service_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  remove_object_service_ = this->create_service<RemoveObjectFromSceneSrv>(
    "remove_object_from_scene",
    std::bind(
      &PlanningSceneHandler::remove_object_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  // select_target_service_ = this->create_service<SelectNextTargetSrv>(
  //   "select_next_target",
  //   std::bind(
  //     &PlanningSceneHandler::select_target_service_callback,
  //     this,
  //     std::placeholders::_1,
  //     std::placeholders::_2));

  update_from_pose_service_ = this->create_service<UpdatePlanningSceneFromPosesSrv>(
    "update_from_pose_scene",
    std::bind(
      &PlanningSceneHandler::update_from_pose_service_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(logger_, "Created.");
}

PlanningSceneHandler::~PlanningSceneHandler()
{}

void PlanningSceneHandler::declare_parameters()
{
  declare_parameter("ground_plane_link", rclcpp::ParameterValue("base_link"));
  declare_parameter("ground_plane_dimension",
    rclcpp::ParameterValue(std::vector<double>({1.0, 1.0, 0.001})));
  declare_parameter("ground_plane_position",
    rclcpp::ParameterValue(std::vector<double>({0.0, 0.0, -0.066})));
  declare_parameter("target_shape", rclcpp::ParameterValue("SPHERE"));
  declare_parameter("target_dimension", rclcpp::ParameterValue(std::vector<double>({0.03})));
}

void PlanningSceneHandler::clear_service_callback(
  const std::shared_ptr<ClearPlanningSceneSrv::Request> /*request*/,
  std::shared_ptr<ClearPlanningSceneSrv::Response> response)
{
  // Dettaches objects.
  auto attached_objects = planning_scene_interface_.getAttachedObjects();
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  for (const auto& attached_object: attached_objects) {
    detach_object.object.id = attached_object.first;
    detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_.applyAttachedCollisionObject(detach_object);
  }

  // Removes collision objects.
  std::vector<std::string> collision_objects = planning_scene_interface_.getKnownObjectNames();
  auto objects_size = collision_objects.size();

  response->success = true;
  if (!objects_size) {
    response->message = "No objects were found in the scene";
    return;
  }

  planning_scene_interface_.removeCollisionObjects(collision_objects);
  response->message = std::to_string(objects_size) + " objects removed from the scene";
}

void PlanningSceneHandler::remove_object_callback(
  const std::shared_ptr<RemoveObjectFromSceneSrv::Request> request,
  std::shared_ptr<RemoveObjectFromSceneSrv::Response> response)
{
  auto attached_objects = planning_scene_interface_.getAttachedObjects();
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  for (const auto& attached_object: attached_objects) {
    if (request->object_name == attached_object.first) {
      detach_object.object.id = attached_object.first;
      detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      planning_scene_interface_.applyAttachedCollisionObject(detach_object);
    }
  }

  planning_scene_interface_.removeCollisionObjects({request->object_name});
  response->success = true;
  response->message = "removed";
}

void PlanningSceneHandler::update_from_pose_service_callback(
  const std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Request> request,
  std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Response> response)
{
  const auto objects_size = request->perception_scene.objects.size();
  const auto objects = request->perception_scene.objects;
  const auto frame_id = request->perception_scene.header.frame_id;

  if (!objects_size) {
    response->success = false;
    response->message = "No objects were added into the scene.";
    return;
  }

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(objects_size+1);

  // Ground plane
  auto ground_plane_position = get_parameter("ground_plane_position").as_double_array();
  geometry_msgs::msg::Pose ground_pose;
  ground_pose.position.x = ground_plane_position[0];
  ground_pose.position.y = ground_plane_position[1];
  ground_pose.position.z = ground_plane_position[2];
  ground_pose.orientation.w = 1;
  add_single_object_to_the_scene(
    collision_objects[0],
    "ground_plane",
    get_parameter("ground_plane_link").as_string(),
    shape_msgs::msg::SolidPrimitive::BOX,
    get_parameter("ground_plane_dimension").as_double_array(),
    ground_pose);

  // Objects
  size_t index = 1;
  for (const auto& object: objects) {
    add_single_object_to_the_scene(
      collision_objects[index],
      object.class_name+"_"+std::to_string(index),
      frame_id,
      target_primitive_,
      get_parameter("target_dimension").as_double_array(),
      object.pose);
    ++index;
  }

  planning_scene_interface_.applyCollisionObjects(collision_objects);

  response->success = true;
  response->message = std::to_string(objects_size) + " objects added into the scene.";
}

}  // namespace sh_planning_scene_handler
