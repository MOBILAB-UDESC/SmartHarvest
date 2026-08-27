#ifndef SH_MOVE_GROUP_SERVER__MOVE_GROUP_SERVER_HPP_
#define SH_MOVE_GROUP_SERVER__MOVE_GROUP_SERVER_HPP_

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/motion_constraint_generator_base.hpp"
#include "sh_interfaces/action/move_to_object.hpp"
#include "sh_interfaces/action/move_to_named_target.hpp"
#include "sh_interfaces/srv/select_next_target.hpp"
#include "sh_move_group_server/moveit_config_loader.hpp"

namespace sh_move_group_server
{

using CallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;
using MoveToNamedTargetAction = sh_interfaces::action::MoveToNamedTarget;
using GoalHandleMoveToNamedTargetAction = rclcpp_action::ServerGoalHandle<MoveToNamedTargetAction>;
using MoveToObjectAction = sh_interfaces::action::MoveToObject;
using GoalHandleMoveToObjectAction = rclcpp_action::ServerGoalHandle<MoveToObjectAction>;
using SelectNextTargetSrv = sh_interfaces::srv::SelectNextTarget;

/**
 * @struct sh_move_group_server::MoveGroupParameter
 * @brief Options for a MoveGroup.
 */
struct MoveGroupParameter
{
  std::string move_group;
  std::string planner_id, planning_pipeline;
  double planning_time;
  int planning_attempts;
  std::vector<double> vel_acc_scaling_factors;
  std::map<std::string, std::string> planner_params;
  bool override;
};  // struct MoveGroupParameter

struct MoveGroupTypes
{
  MoveGroupParameter arm;
  MoveGroupParameter gripper;
};  // struct MoveGroupTypes

/**
 * @class sh_move_group_server::MoveGroupServer
 * @brief ROS 2 action server node that executes MoveIt plans for three target types:
 * named_target (defined in SRDF), object_name (collision object in planning scene),
 * and explicit Pose.
 *
 * This node exposes one action server per target type and reuses a
 * common planning and execution pipeline.
 */
class MoveGroupServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief A constructor for sh_move_group_server::MoveGroupServer class.
   *
   * @param node_name Name of the node.
   */
  explicit MoveGroupServer(const std::string& node_name);

  /**
   * @brief A destructor for sh_move_group_server::MoveGroupServer class.
   */
  ~MoveGroupServer();

private:
  /**
   * @brief Callback function for configure transition.
   *
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for activate transition.
   *
   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for deactivate transition.

   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for cleanup transition.

   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Callback function for shutdown transition.

   * @param state A reference to the state of the Lifecycle Node.
   * @return SUCCESS or FAILURE.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  MoveItConfigOptions get_moveit_config_options();

  /**
   * @brief Copy the MoveIt configuration parameters exposed by the move_group node
   * (kinematics solvers, joint limits) onto node_.
   *
   * MoveGroupInterface builds its RobotModel from node_, and RobotModelLoader only
   * instantiates the kinematics solvers of a group when the matching
   * robot_description_kinematics.<group>.* parameters live on that node. Without them
   * planning through move_group still works (it is solved server side), but every client
   * side IK query, such as the setFromIK call of the cartesian pipeline, fails with
   * "No kinematics solver instantiated for group". The RobotModel is built and cached on
   * the first MoveGroupInterface construction, so this has to run before
   * arm_move_group_init().
   *
   * @return True when at least one parameter was imported.
   */
  bool import_moveit_parameters();

  /**
   * @brief Block until a node shows up in the ROS graph.
   *
   * @param fully_qualified_name Name of the node to wait for, leading slash included.
   * @param deadline Absolute time at which to give up.
   * @return True when the node showed up before the deadline.
   */
  bool wait_for_node(
    const std::string& fully_qualified_name,
    const std::chrono::steady_clock::time_point& deadline);

  /**
   * @brief Initialize Moveit Group Interface for the arm
   */
  void arm_move_group_init();

  /**
   * @brief Initialize Moveit Group Interface for the gripper
   */
  void gripper_move_group_init();

  void select_target_service_callback(
    const std::shared_ptr<SelectNextTargetSrv::Request> /*request*/,
    std::shared_ptr<SelectNextTargetSrv::Response> response);

  /**
   * @brief Move to named target action setup.
   */
  void setup_move_to_named_target_action();

  /**
   * @brief Move to object action setup.
   */
  void setup_move_to_object_action();

  /**
   * @brief Check whether a named target exists in the SRDF for the active move group.
   *
   * @param named_target Named target to validate.
   * @param move_group Named planning group.
   * @return True when the named target exists.
   */
  bool named_target_exists(const std::string& named_target, const std::string& move_group);

  /**
   * @brief Check whether an object exists in the planning scene.
   *
   * @param object_name Named of the collision object to validate.
   * @return True when the collision object exists.
   */
  bool object_exists(const std::string& object_name);

  void get_pose_from_named_target(const std::string& named_target, geometry_msgs::msg::Pose& goal_pose);

  /**
   * @brief Select the move group interface.
   *
   * @param move_group Named planning group.
   */
  bool select_move_group(const std::string& move_group);

  /**
   * @brief Abstract method for planning and execution routine for all action types.
   *
   * @tparam MoveAction Action type.
   * @param goal_handle Handle of accepted goal.
   * @param result Action result object to fill.
   * @return true or false.
   */
  template <class MoveAction>
  bool plan_and_execute(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveAction>> goal_handle,
    std::shared_ptr<typename MoveAction::Result>& result);

  template <class MoveAction>
  bool plan_and_execute_cartesian(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveAction>> goal_handle,
    std::shared_ptr<typename MoveAction::Result>& result,
    geometry_msgs::msg::Pose& pre_goal_pose,
    geometry_msgs::msg::Pose& goal_pose);

  geometry_msgs::msg::Pose compute_pre_grasp(
    const geometry_msgs::msg::Pose& grasp_pose,
    double offset_m);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;

  // Planning groups
  MoveGroupTypes move_group_types_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> selected_move_group_;
  std::string goal_group_name_;

  // Plugins
  pluginlib::ClassLoader<sh_base_template::MotionConstraintGeneratorBase> constraint_loader_;
  // std::string detector_plugin_name_;
  pluginlib::UniquePtr<sh_base_template::MotionConstraintGeneratorBase> constraints_;

  // Planning scene access
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<std::string> scene_objects_name_;

  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;

  // NAMED TARGET ACTION
  rclcpp_action::Server<MoveToNamedTargetAction>::SharedPtr move_to_named_target_server_;
  std::string goal_named_target_;
  rclcpp_action::Server<MoveToObjectAction>::SharedPtr move_to_object_server_;
  std::string goal_object_name_;
  bool attach_, detach_;
  std::string object_to_attach_detach_, link_to_attach_detach_;

  rclcpp::Service<SelectNextTargetSrv>::SharedPtr select_target_service_;

  bool use_constraints_;
  bool cartesian_;

};

}  // namespace sh_move_group_server

#endif  // SH_MOVE_GROUP_SERVER__MOVE_GROUP_SERVER_HPP_