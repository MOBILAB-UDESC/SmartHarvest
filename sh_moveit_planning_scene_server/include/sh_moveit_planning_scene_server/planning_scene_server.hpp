#ifndef SH_MOVEIT_PLANNING_SCENE_SERVER__PLANNING_SCENE_SERVER_HPP_
#define SH_MOVEIT_PLANNING_SCENE_SERVER__PLANNING_SCENE_SERVER_HPP_

#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sh_interfaces/srv/clear_planning_scene.hpp"
#include "sh_interfaces/srv/update_planning_scene_from_poses.hpp"
#include "sh_moveit_planning_scene_server/planning_scene_utils.hpp"

namespace sh_moveit_planning_scene_server
{

using ClearPlanningSceneSrv = sh_interfaces::srv::ClearPlanningScene;
using UpdatePlanningSceneFromPosesSrv = sh_interfaces::srv::UpdatePlanningSceneFromPoses;

class PlanningSceneServer : public rclcpp::Node
{
public:
  explicit PlanningSceneServer(const std::string& server_name);

  ~PlanningSceneServer();

private:

  void declare_parameters();

  void clear_service_callback(
    const std::shared_ptr<ClearPlanningSceneSrv::Request> /*request*/,
    std::shared_ptr<ClearPlanningSceneSrv::Response> response);

  void update_from_pose_service_callback(
    const std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Request> request,
    std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Response> response);

  rclcpp::Logger logger_;
  rclcpp::Service<ClearPlanningSceneSrv>::SharedPtr clear_service_;
  rclcpp::Service<UpdatePlanningSceneFromPosesSrv>::SharedPtr update_from_pose_service_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  uint8_t target_primitive_;
};

}  // namespace sh_moveit_planning_scene_server

#endif  // SH_MOVEIT_PLANNING_SCENE_SERVER__PLANNING_SCENE_SERVER_HPP_