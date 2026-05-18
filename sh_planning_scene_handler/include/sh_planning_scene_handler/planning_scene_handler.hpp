#ifndef SH_PLANNING_SCENE_HANDLER__PLANNING_SCENE_HANDLER_HPP_
#define SH_PLANNING_SCENE_HANDLER__PLANNING_SCENE_HANDLER_HPP_

#include <memory>
#include <string>

#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sh_interfaces/srv/clear_planning_scene.hpp"
// #include "sh_interfaces/srv/select_next_target.hpp"
#include "sh_interfaces/srv/update_planning_scene_from_poses.hpp"

namespace sh_planning_scene_handler
{

using ClearPlanningSceneSrv = sh_interfaces::srv::ClearPlanningScene;
// using SelectNextTargetSrv = sh_interfaces::srv::SelectNextTarget;
using UpdatePlanningSceneFromPosesSrv = sh_interfaces::srv::UpdatePlanningSceneFromPoses;

class PlanningSceneHandler : public rclcpp::Node
{
public:
  explicit PlanningSceneHandler(const std::string& server_name);

  ~PlanningSceneHandler();

private:

  void declare_parameters();

  void clear_service_callback(
    const std::shared_ptr<ClearPlanningSceneSrv::Request> /*request*/,
    std::shared_ptr<ClearPlanningSceneSrv::Response> response);

  // void select_target_service_callback(
  //   const std::shared_ptr<SelectNextTargetSrv::Request> /*request*/,
  //   std::shared_ptr<SelectNextTargetSrv::Response> response);

  void update_from_pose_service_callback(
    const std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Request> request,
    std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Response> response);

  rclcpp::Logger logger_;
  rclcpp::Service<ClearPlanningSceneSrv>::SharedPtr clear_service_;
  // rclcpp::Service<SelectNextTargetSrv>::SharedPtr select_target_service_;
  rclcpp::Service<UpdatePlanningSceneFromPosesSrv>::SharedPtr update_from_pose_service_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  uint8_t target_primitive_;
};

}  // namespace sh_planning_scene_handler

#endif  // SH_PLANNING_SCENE_HANDLER__PLANNING_SCENE_HANDLER_HPP_