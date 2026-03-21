#include "sh_moveit_planning_scene_server/planning_scene_server.hpp"

namespace sh_moveit_planning_scene_server
{

PlanningSceneServer::PlanningSceneServer(const std::string& server_name) :
  rclcpp::Node(server_name),
  logger_(rclcpp::get_logger(server_name))
{
  RCLCPP_INFO(logger_, "Creating.");

  declare_parameters();
  target_primitive_ = get_primitive(get_parameter("target_shape").as_string());

  clear_service_ = this->create_service<ClearPlanningSceneSrv>(
    get_parameter("clear_scene_service_name").as_string(),
    std::bind(
      &PlanningSceneServer::clear_service_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  update_from_pose_service_ = this->create_service<UpdatePlanningSceneFromPosesSrv>(
    get_parameter("update_scene_service_name").as_string(),
    std::bind(
      &PlanningSceneServer::update_from_pose_service_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(logger_, "Created.");
}

PlanningSceneServer::~PlanningSceneServer()
{}

void PlanningSceneServer::declare_parameters()
{
  declare_parameter("clear_scene_service_name", rclcpp::ParameterValue("clear_scene"));
  declare_parameter("update_scene_service_name", rclcpp::ParameterValue("update_from_pose_scene"));
  declare_parameter("ground_plane_link", rclcpp::ParameterValue("base_link"));
  declare_parameter("ground_plane_dimension",
    rclcpp::ParameterValue(std::vector<double>({1.0, 1.0, 0.001})));
  declare_parameter("ground_plane_position",
    rclcpp::ParameterValue(std::vector<double>({0.0, 0.0, -0.066})));
  declare_parameter("target_shape", rclcpp::ParameterValue("SPHERE"));
  declare_parameter("target_dimension", rclcpp::ParameterValue(std::vector<double>({0.03})));
}

void PlanningSceneServer::clear_service_callback(
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

void PlanningSceneServer::update_from_pose_service_callback(
  const std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Request> request,
  std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Response> response)
{
  const auto objects_size = request->detected_objects.num_objects;
  const auto objects = request->detected_objects.objects;
  const auto frame_id = request->detected_objects.header.frame_id;

  if (!objects_size) {
    response->success = false;
    response->message = "No objects were added into the scene.";
    return;
  }

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(objects_size+1);

  // Ground plane
  add_single_object_to_the_scene(
    collision_objects[0],
    "ground_plane",
    get_parameter("ground_plane_link").as_string(),
    shape_msgs::msg::SolidPrimitive::BOX,
    get_parameter("ground_plane_dimension").as_double_array(),
    get_parameter("ground_plane_position").as_double_array(),
    {0.0, 0.0, 0.0, 1.0});

  // Objects
  size_t index = 1;
  for (const auto& object: objects) {
    add_single_object_to_the_scene(
      collision_objects[index],
      "Apple_"+std::to_string(index),
      frame_id,
      target_primitive_,
      get_parameter("target_dimension").as_double_array(),
      {object.x, object.y, object.z},
      {object.grasp_orientation_quaternion[0],
        object.grasp_orientation_quaternion[1],
        object.grasp_orientation_quaternion[2],
        object.grasp_orientation_quaternion[3]});
    ++index;
  }

  planning_scene_interface_.applyCollisionObjects(collision_objects);

  response->success = true;
  response->message = std::to_string(objects_size) + " objects added into the scene.";
}

}  // namespace sh_moveit_planning_scene_server
