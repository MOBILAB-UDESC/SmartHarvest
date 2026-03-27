#ifndef SH_MOVE_GROUP_SERVER__MOVE_GROUP_SERVER_HPP_
#define SH_MOVE_GROUP_SERVER__MOVE_GROUP_SERVER_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sh_interfaces/action/move_to_named_target.hpp"
#include "sh_interfaces/action/move_to_object.hpp"
#include "sh_interfaces/action/move_to_pose.hpp"
#include "sh_move_group_server/parameter_handler.hpp"

namespace sh_move_group_server
{
using moveit::planning_interface::MoveGroupInterface;
using MoveToNamedTargetAction = sh_interfaces::action::MoveToNamedTarget;
using GoalHandleMoveToNamedTargetAction = rclcpp_action::ServerGoalHandle<MoveToNamedTargetAction>;
using MoveToPoseAction = sh_interfaces::action::MoveToPose;
using GoalHandleMoveToPoseAction = rclcpp_action::ServerGoalHandle<MoveToPoseAction>;
using MoveToObjectAction = sh_interfaces::action::MoveToObject;
using GoalHandleMoveToObjectAction = rclcpp_action::ServerGoalHandle<MoveToObjectAction>;

/**
 * @brief Final Goal status reported by the action
 */
enum GoalStatus {SUCCEEDED = uint8_t(0), ABORTED = uint8_t(1), CANCELED = uint8_t(2)};
/**
 * @brief Planning/Execution phase reported in action feedback
 */
enum FeedBackStatus {PLANNING = uint8_t(0), EXECUTING = uint8_t(1)};

/**
 * @class sh_move_group_server::MoveGroupServer
 * @brief ROS 2 action server node that executes MoveIt plans for three target types:
 * named_target (defined in SRDF), object_name (collision object in planning scene),
 * and explicit Pose.
 *
 * This node exposes one action server per target type and reuses a
 * common planning and execution pipeline.
 */
class MoveGroupServer : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for sh_move_group_server::MoveGroupServer class.
   *
   * Initializes action servers, and Moveit Group Interfaces.
   * @param node_name Name of the node.
   * @param options ROS 2 node options.
   */
  explicit MoveGroupServer(const std::string& node_name, const rclcpp::NodeOptions& options);

  /**
   * @brief A destructor for sh_move_group_server::MoveGroupServer class.
   */
  ~MoveGroupServer();

private:
  /**
   * @brief Declare and initialize parameters
   */
  void update_parameters();

  /**
   * @brief Initialize Moveit Group Interface for the arm
   */
  void arm_move_group_init();

  /**
   * @brief Initialize Moveit Group Interface for the gripper
   */
  void gripper_move_group_init();

  /**
   * @brief Executo an accepted MoveToNameTarget goal asynchronously.
   *
   * @param goal_handle Handle for the accepted goal.
   */
  void handle_move_to_named_target_accepted(
    std::shared_ptr<GoalHandleMoveToNamedTargetAction> goal_handle);

  /**
   * @brief Executo an accepted MoveToObject goal asynchronously.
   *
   * @param goal_handle Handle for the accepted goal.
   */
  void handle_move_to_object_accepted(
    std::shared_ptr<GoalHandleMoveToObjectAction> goal_handle);

  /**
   * @brief Executo an accepted MoveToPose goal asynchronously.
   *
   * @param goal_handle Handle for the accepted goal.
   */
  void handle_move_to_pose_accepted(
    std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle);

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

  /**
   * @brief Retrieve the pose of the target object from the planning scene.
   * @return Object pose.
  */
  geometry_msgs::msg::Pose get_object_pose();

  /**
   * @brief Check if a pose is all zeros.
   *
   * @param pose Pose to inspect.
   * @return True when all position/orientation components are zero.
   */
  bool is_zero_pose(const geometry_msgs::msg::Pose& pose);

  /**
   * @brief Select the move group interface.
   *
   * @param move_group Named planning group.
   */
  void select_move_group(const std::string& move_group);

  /**
   * @brief Abstract method for planning and execution routine for all action types.
   *
   * @tparam MoveAction Action type.
   * @param goal_handle Handle of accepted goal.
   * @param result Action result object to fill.
   * @return GoalStatus final state.
   */
  template <class MoveAction>
  GoalStatus plan_and_execute(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveAction>> goal_handle,
    std::shared_ptr<typename MoveAction::Result>& result);

  // ROS node handles
  rclcpp::Logger logger_;
  rclcpp::Node::SharedPtr node_;

  // Action servers
  rclcpp_action::Server<MoveToNamedTargetAction>::SharedPtr move_to_named_target_server_;
  rclcpp_action::Server<MoveToObjectAction>::SharedPtr move_to_object_server_;
  rclcpp_action::Server<MoveToPoseAction>::SharedPtr move_to_pose_server_;

  // Planning groups
  std::shared_ptr<MoveGroupInterface> arm_move_group_;
  std::shared_ptr<MoveGroupInterface> gripper_move_group_;
  std::shared_ptr<MoveGroupInterface> selected_move_group_;

  // Cached goal data captured from action requests.
  std::string goal_named_target_;
  std::string goal_object_name_;
  std::string goal_move_group_;
  geometry_msgs::msg::Pose object_pose_;
  geometry_msgs::msg::Pose pose_;
  bool attach_, detach_;
  std::string object_to_attach_detach_, link_to_attach_detach_;

  // Planning scene access
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<std::string> scene_objects_name_;

  // Node parameters and mutex for synchronized access to them
  Parameter params_;
  std::map<std::string, std::string> arm_planner_map_;
  std::map<std::string, std::string> gripper_planner_map_;
  std::mutex param_mutex_;
};

}  // sh_move_group_server

#endif  // SH_MOVE_GROUP_SERVER__MOVE_GROUP_SERVER_HPP_