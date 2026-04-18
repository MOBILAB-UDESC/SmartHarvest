#include "sh_move_group_server/move_group_server.hpp"

namespace sh_move_group_server
{

MoveGroupServer::MoveGroupServer(const std::string& node_name, const rclcpp::NodeOptions& options) :
  rclcpp::Node(node_name, options),
  logger_(rclcpp::get_logger(node_name)),
  node_(std::make_shared<rclcpp::Node>("move_group_server_node")),
  executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
  std::string move_to_named_target_action_name, move_to_pose_action_name, move_to_object_action_name;
  this->get_parameter_or<std::string>(
    "action_names.move_to_named_target", move_to_named_target_action_name, "move_to_named_target");
  this->get_parameter_or<std::string>(
    "action_names.move_to_pose", move_to_pose_action_name, "move_to_pose");
  this->get_parameter_or<std::string>(
    "action_names.move_to_object", move_to_object_action_name, "move_to_object");

  executor_->add_node(node_);
  std::thread([this]() { executor_->spin(); }).detach();

  update_parameters();
  arm_move_group_init();
  gripper_move_group_init();
  selected_move_group_ = arm_move_group_;

  clear_planning_constraints();

  using namespace std::placeholders;
  move_to_named_target_server_ = rclcpp_action::create_server<MoveToNamedTargetAction>(
    this,
    move_to_named_target_action_name,
    // Goal request
    [this](
      const rclcpp_action::GoalUUID& /*uuid*/,
      std::shared_ptr<const MoveToNamedTargetAction::Goal> goal)
    {
      goal_named_target_ = goal->named_target;
      goal_move_group_ = goal->group_name;
      attach_ = goal->attach;
      detach_ = goal->detach;
      object_to_attach_detach_ = goal->object_to_attach_detach;
      link_to_attach_detach_ = goal->link_to_attach_detach;

      if (!named_target_exists(goal_named_target_, goal_move_group_)) {
        RCLCPP_ERROR(
          logger_, "%s is not a valid target. Check your SRDF file.", goal_named_target_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(logger_, "%s target found.", goal_named_target_.c_str());
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // Goal cancel
    [this](std::shared_ptr<GoalHandleMoveToNamedTargetAction> /*goal_handle*/)
    {
      RCLCPP_INFO(logger_, "Received request to cancel goal");
      selected_move_group_->stop();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // Goal execution
    std::bind(&MoveGroupServer::handle_move_to_named_target_accepted, this, _1));

  move_to_object_server_ = rclcpp_action::create_server<MoveToObjectAction>(
    this,
    move_to_object_action_name,
    // Goal request
    [this](
      const rclcpp_action::GoalUUID& /*uuid*/,
      std::shared_ptr<const MoveToObjectAction::Goal> goal)
    {
      goal_object_name_ = goal->object_name;
      goal_move_group_ = goal->group_name;

      if (!object_exists(goal_object_name_)) {
        RCLCPP_ERROR(
          logger_, "%s is not a valid object.", goal_object_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(logger_, "%s object found.", goal_object_name_.c_str());
      object_pose_ = planning_scene_interface_.getObjectPoses({goal_object_name_})[goal_object_name_];
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // Goal cancel
    [this](std::shared_ptr<GoalHandleMoveToObjectAction> /*goal_handle*/)
    {
      RCLCPP_INFO(logger_, "Received request to cancel goal.");
      selected_move_group_->stop();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // Goal execution
    std::bind(&MoveGroupServer::handle_move_to_object_accepted, this, _1));

  move_to_pose_server_ = rclcpp_action::create_server<MoveToPoseAction>(
    this,
    move_to_pose_action_name,
    // Goal request
    [this](
      const rclcpp_action::GoalUUID& /*uuid*/,
      std::shared_ptr<const MoveToPoseAction::Goal> goal)
    {
      pose_ = goal->target_pose;
      goal_move_group_ = goal->group_name;

      if (is_zero_pose(pose_)) {
        RCLCPP_ERROR(logger_, "Pose is all zero.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(logger_, "Pose saved.");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // Goal cancel
    [this](std::shared_ptr<GoalHandleMoveToPoseAction> /*goal_handle*/)
    {
      RCLCPP_INFO(logger_, "Received request to cancel goal.");
      selected_move_group_->stop();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // Goal execution
    std::bind(&MoveGroupServer::handle_move_to_pose_accepted, this, _1));

  RCLCPP_INFO(logger_, "Created.");
}

MoveGroupServer::~MoveGroupServer()
{}

void MoveGroupServer::update_parameters()
{
  this->get_parameter_or<std::string>(
    "arm_planning_config.move_group", params_.arm.move_group, "arm");
  this->get_parameter_or<std::vector<double>>(
    "arm_planning_config.vel_acc_scaling_factors",
    params_.arm.vel_acc_scaling_factors,
    std::vector<double>({0.6, 0.6}));
  this->get_parameter_or<std::string>(
    "arm_planning_config.planning_pipeline", params_.arm.planning_pipeline, "ompl");
  this->get_parameter_or<std::string>(
    "arm_planning_config.planner_id", params_.arm.planner_id, "RRTConnect");
  this->get_parameter_or<double>(
    "arm_planning_config.planning_time", params_.arm.planning_time, 5.0);
  this->get_parameter_or<int>(
    "arm_planning_config.planning_attempts", params_.arm.planning_attempts, 5);
  this->get_parameter_or<bool>(
    "arm_planning_config.override", params_.arm.override, false);
  this->get_parameters<std::string>(
    "arm_planning_config."+params_.arm.planner_id, params_.arm.planner_params);

  this->get_parameter_or<std::string>(
    "gripper_planning_config.move_group", params_.gripper.move_group, "gripper");
  this->get_parameter_or<std::vector<double>>(
    "gripper_planning_config.vel_acc_scaling_factors",
    params_.gripper.vel_acc_scaling_factors,
    std::vector<double>({0.6, 0.6}));
  this->get_parameter_or<std::string>(
    "gripper_planning_config.planning_pipeline", params_.gripper.planning_pipeline, "ompl");
  this->get_parameter_or<std::string>(
    "gripper_planning_config.planner_id", params_.gripper.planner_id, "RRTConnect");
  this->get_parameter_or<double>(
    "gripper_planning_config.planning_time", params_.gripper.planning_time, 5.0);
  this->get_parameter_or<int>(
    "gripper_planning_config.planning_attempts", params_.gripper.planning_attempts, 5);
  this->get_parameter_or<bool>(
    "gripper_planning_config.override", params_.gripper.override, false);
  this->get_parameters<std::string>(
    "gripper_planning_config."+params_.gripper.planner_id, params_.gripper.planner_params);
}

void MoveGroupServer::arm_move_group_init()
{
  arm_move_group_ = std::make_shared<MoveGroupInterface>(node_, params_.arm.move_group);

  arm_move_group_->setPlanningPipelineId(params_.arm.planning_pipeline);
  arm_move_group_->setPlannerId(params_.arm.planner_id);
  arm_move_group_->setPlanningTime(params_.arm.planning_time);
  arm_move_group_->setReplanAttempts(params_.arm.planning_attempts);
  arm_move_group_->setMaxVelocityScalingFactor(params_.arm.vel_acc_scaling_factors[0]);
  arm_move_group_->setMaxAccelerationScalingFactor(params_.arm.vel_acc_scaling_factors[1]);

  arm_move_group_->setPlannerParams(
    params_.arm.planner_id, params_.arm.move_group, params_.arm.planner_params, false);

  moveit_visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
    node_, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, arm_move_group_->getRobotModel());
}

void MoveGroupServer::gripper_move_group_init()
{
  gripper_move_group_ = std::make_shared<MoveGroupInterface>(node_, params_.gripper.move_group);

  gripper_move_group_->setPlanningPipelineId(params_.gripper.planning_pipeline);
  gripper_move_group_->setPlannerId(params_.gripper.planner_id);
  gripper_move_group_->setPlanningTime(params_.gripper.planning_time);
  gripper_move_group_->setReplanAttempts(params_.gripper.planning_attempts);
  gripper_move_group_->setMaxVelocityScalingFactor(params_.gripper.vel_acc_scaling_factors[0]);
  gripper_move_group_->setMaxAccelerationScalingFactor(params_.gripper.vel_acc_scaling_factors[1]);

  gripper_move_group_->setPlannerParams(
    params_.gripper.planner_id, params_.gripper.move_group, params_.gripper.planner_params, false);
}

void MoveGroupServer::handle_move_to_named_target_accepted(
  std::shared_ptr<GoalHandleMoveToNamedTargetAction> goal_handle)
{
  std::thread([this](std::shared_ptr<GoalHandleMoveToNamedTargetAction> goal_handle,
    const bool attach, const bool detach,
    const std::string object_to_attach_detach, const std::string link_to_attach_detach) {

    clear_planning_constraints();

    auto result = std::make_shared<MoveToNamedTargetAction::Result>();
    select_move_group(goal_move_group_);

    if (attach) {
      selected_move_group_->attachObject(
        object_to_attach_detach, link_to_attach_detach, selected_move_group_->getLinkNames());
    }
    if (detach) {
      selected_move_group_->detachObject(object_to_attach_detach);
      planning_scene_interface_.removeCollisionObjects({object_to_attach_detach});
    }

    selected_move_group_->setNamedTarget(goal_named_target_);

    auto plan_and_execute_result = plan_and_execute<MoveToNamedTargetAction>(
      goal_handle, result);

    if (plan_and_execute_result == GoalStatus::ABORTED) {
      result->success = false;
      if (goal_handle->is_executing()) {
        goal_handle->abort(result);
      }
      return;
    } else if (plan_and_execute_result == GoalStatus::CANCELED) {
      return;
    }

    // When plan_and_execute_result == GoalStatus::SUCCEEDED

    result->success = true;
    goal_handle->succeed(result);
  }, goal_handle, attach_, detach_, object_to_attach_detach_, link_to_attach_detach_).detach();
}

void MoveGroupServer::handle_move_to_object_accepted(
  std::shared_ptr<GoalHandleMoveToObjectAction> goal_handle)
{
  std::thread([this](std::shared_ptr<GoalHandleMoveToObjectAction> goal_handle) {
    auto result = std::make_shared<MoveToObjectAction::Result>();
    select_move_group(goal_move_group_);
    selected_move_group_->setPoseTarget(object_pose_);

    clear_planning_constraints();
    update_planning_constraints();

    auto plan_and_execute_result = plan_and_execute<MoveToObjectAction>(goal_handle, result);

    if (plan_and_execute_result == GoalStatus::ABORTED) {
      result->success = false;
      if (goal_handle->is_executing()) {
        goal_handle->abort(result);
      }
      return;
    } else if (plan_and_execute_result == GoalStatus::CANCELED) {
      return;
    }

    // When plan_and_execute_result == GoalStatus::SUCCEEDED
    result->success = true;
    goal_handle->succeed(result);
  }, goal_handle).detach();
}

void MoveGroupServer::handle_move_to_pose_accepted(
  std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle)
{
  std::thread([this](std::shared_ptr<GoalHandleMoveToPoseAction> goal_handle) {

    clear_planning_constraints();

    auto result = std::make_shared<MoveToPoseAction::Result>();
    select_move_group(goal_move_group_);
    selected_move_group_->setPoseTarget(pose_);

    auto plan_and_execute_result = plan_and_execute<MoveToPoseAction>(goal_handle, result);

    if (plan_and_execute_result == GoalStatus::ABORTED) {
      result->success = false;
      if (goal_handle->is_executing()) {
        goal_handle->abort(result);
      }
      return;
    } else if (plan_and_execute_result == GoalStatus::CANCELED) {
      return;
    }

    // When plan_and_execute_result == GoalStatus::SUCCEEDED
    result->success = true;
    goal_handle->succeed(result);
  }, goal_handle).detach();
}

bool MoveGroupServer::named_target_exists(
  const std::string& named_target,
  const std::string& move_group)
{
  std::vector<std::string> named_targets;
  {
    std::lock_guard<std::mutex> lock(param_mutex_);
    if (move_group == params_.arm.move_group) {
      named_targets = arm_move_group_->getNamedTargets();
    } else if (move_group == params_.gripper.move_group) {
      named_targets = gripper_move_group_->getNamedTargets();
    } else {
      return false;
    }
  }
  auto result = std::find(named_targets.begin(), named_targets.end(), named_target);
  if (result != named_targets.end()) {
    return true;
  }
  return false;
}

bool MoveGroupServer::object_exists(const std::string& object_name)
{
  std::vector<std::string> object_names = planning_scene_interface_.getKnownObjectNames();
  auto result = std::find(object_names.begin(), object_names.end(), object_name);
  if (result != object_names.end()) {
    return true;
  }
  return false;
}

bool MoveGroupServer::is_zero_pose(const geometry_msgs::msg::Pose& pose)
{
  return pose.position.x == 0.0 &&
         pose.position.y == 0.0 &&
         pose.position.z == 0.0 &&
         pose.orientation.x == 0.0 &&
         pose.orientation.y == 0.0 &&
         pose.orientation.z == 0.0 &&
         pose.orientation.w == 0.0;
}

void MoveGroupServer::select_move_group(const std::string& move_group)
{
  std::lock_guard<std::mutex> lock(param_mutex_);
  if (move_group == params_.arm.move_group) {
    selected_move_group_ = arm_move_group_;
  } else {
    selected_move_group_ = gripper_move_group_;
  }
}

void MoveGroupServer::clear_planning_constraints()
{
  selected_move_group_->clearPathConstraints();
  moveit_visual_tools_->deleteAllMarkers();
  moveit_visual_tools_->trigger();
}

void MoveGroupServer::update_planning_constraints()
{
  auto current_pose = selected_move_group_->getCurrentPose(selected_move_group_->getEndEffectorLink());
  moveit_visual_tools_->publishSphere(current_pose.pose, rviz_visual_tools::RED, 0.05);
  moveit_visual_tools_->publishSphere(object_pose_, rviz_visual_tools::GREEN, 0.05);

  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = selected_move_group_->getPoseReferenceFrame();
  box_constraint.link_name = selected_move_group_->getEndEffectorLink();
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.1, 0.1, 0.1 };
  box.dimensions[0] += 2*std::abs(object_pose_.position.x - current_pose.pose.position.x);
  box.dimensions[1] += 2*std::abs(object_pose_.position.y - current_pose.pose.position.y);
  box.dimensions[2] += 2*std::abs(object_pose_.position.z - current_pose.pose.position.z);

  box_constraint.constraint_region.primitives.emplace_back(box);

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = current_pose.pose.position.x;
  box_pose.position.y = current_pose.pose.position.y;
  box_pose.position.z = current_pose.pose.position.z;
  box_pose.orientation.w = 1.0;

  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;

  Eigen::Vector3d new_box_point_1(box_pose.position.x - box.dimensions[0] / 2,
                                  box_pose.position.y - box.dimensions[1] / 2,
                                  box_pose.position.z - box.dimensions[2] / 2);
  Eigen::Vector3d new_box_point_2(box_pose.position.x + box.dimensions[0] / 2,
                                  box_pose.position.y + box.dimensions[1] / 2,
                                  box_pose.position.z + box.dimensions[2] / 2);

  moveit_visual_tools_->publishCuboid(new_box_point_1, new_box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);

  moveit_visual_tools_->trigger();

  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);

  selected_move_group_->setPathConstraints(box_constraints);
  // arm_move_group_->setPlannerId(params_.arm.planner_id);
}

template <class MoveAction>
GoalStatus MoveGroupServer::plan_and_execute(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveAction>> goal_handle,
  std::shared_ptr<typename MoveAction::Result>& result)
{
  auto feedback = std::make_shared<typename MoveAction::Feedback>();

  moveit::planning_interface::MoveGroupInterface::Plan group_plan;
  feedback->phase = FeedBackStatus::PLANNING;
  feedback->message = "Planning started.";
  goal_handle->publish_feedback(feedback);

  auto const ok = static_cast<bool>(selected_move_group_->plan(group_plan));
  auto const [success, plan] = std::make_pair(ok, group_plan);

  if (!success) {
    RCLCPP_ERROR(logger_, "Planning failed!.");
    result->message = "Planning goes wrong.";
    return GoalStatus::ABORTED;
  }

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    return GoalStatus::CANCELED;
  }

  feedback->phase = FeedBackStatus::EXECUTING;
  feedback->message = "Path found.";
  goal_handle->publish_feedback(feedback);
  // auto execution_result = selected_move_group_->execute(plan);

  // if (!execution_result) {
  //   if (goal_handle->is_canceling()) {
  //     result->success = false;
  //     result->message = "Goal canceled.";
  //     goal_handle->canceled(result);
  //     return GoalStatus::CANCELED;
  //   }
  //   RCLCPP_ERROR(logger_, "Path execution failed!. Skipped.");
  //   result->message = "Path execution failed.";
  //   return GoalStatus::ABORTED;
  // }

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    return GoalStatus::CANCELED;
  }

  result->message = "Path execution completed.";
  RCLCPP_INFO(logger_, "Path execution completed.");
  return GoalStatus::SUCCEEDED;
}

}  // namespace sh_move_group_server
