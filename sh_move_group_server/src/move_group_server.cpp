#include "sh_move_group_server/move_group_server.hpp"

#include <thread>

#include "tf2_eigen/tf2_eigen.hpp"

#include "sh_utils/ros2_node_utils.hpp"

// TODO: Add mutex

namespace sh_move_group_server
{

MoveGroupServer::MoveGroupServer(const std::string & node_name) :
  rclcpp_lifecycle::LifecycleNode(node_name),
  node_(std::make_shared<rclcpp::Node>("move_group_server_node")),
  executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
  constraint_loader_("sh_base_template", "sh_base_template::MotionConstraintGeneratorBase")
{
  executor_->add_node(node_);
  std::thread([this]() { executor_->spin(); }).detach();

  declare_parameter<std::string>("base_link", "base_link");
}

MoveGroupServer::~MoveGroupServer()
{

}

CallbackReturn MoveGroupServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(get_logger(), "Configuring.");

  arm_move_group_init();
  gripper_move_group_init();

  select_target_service_ = create_service<SelectNextTargetSrv>(
    "select_next_target",
    std::bind(
      &MoveGroupServer::select_target_service_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  std::string constraint_plugin_name;
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_from_this(), "constraints.plugin",
    constraint_plugin_name, "sh_basic_moveit_pipeline::PrismConstraintGenerator");

  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_from_this(), "constraints.plugin",
    constraint_plugin_name, "sh_basic_moveit_pipeline::PrismConstraintGenerator");

  try {
    constraints_ = constraint_loader_.createUniqueInstance(constraint_plugin_name);
    RCLCPP_INFO(get_logger() , "\033[1;32m[%s] loaded.\033[0m", constraint_plugin_name.c_str());
  } catch (const pluginlib::PluginlibException& e) {
    RCLCPP_ERROR(get_logger(), "Error loading [%s]: ", e.what());
    on_cleanup(state);
    return CallbackReturn::FAILURE;
  }

  if (!constraints_->configure(weak_from_this())) {
    RCLCPP_ERROR(get_logger(), "Constraints plugin configuration failed.");
    on_cleanup(state);
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MoveGroupServer::on_activate(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(get_logger(), "Activating.");

  constraints_->activate();

  // ACTIONS
  setup_move_to_named_target_action();
  setup_move_to_object_action();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MoveGroupServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(get_logger(), "Deactivating.");

  constraints_->activate();

  move_to_named_target_server_.reset();
  move_to_object_server_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MoveGroupServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(get_logger(), "Cleaning up.");

  constraints_->cleanup();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MoveGroupServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(get_logger(), "Shutting down.");
  return CallbackReturn::SUCCESS;
}

void MoveGroupServer::arm_move_group_init()
{
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_from_this(), "arm_planning_config.move_group",
    move_group_types_.arm.move_group, "arm");
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::vector<double>>(
    shared_from_this(), "arm_planning_config.vel_acc_scaling_factors",
    move_group_types_.arm.vel_acc_scaling_factors, std::vector<double>({0.6, 0.6}));
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_from_this(), "arm_planning_config.planning_pipeline",
    move_group_types_.arm.planning_pipeline, "ompl");
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_from_this(), "arm_planning_config.planner_id",
    move_group_types_.arm.planner_id, "RRTConnect");
  sh_utils::ros2_node_utils::declare_and_get_parameter<double>(
    shared_from_this(), "arm_planning_config.planning_time",
    move_group_types_.arm.planning_time, 5.0);
  sh_utils::ros2_node_utils::declare_and_get_parameter<int>(
    shared_from_this(), "arm_planning_config.planning_attempts",
    move_group_types_.arm.planning_attempts, 5);
  sh_utils::ros2_node_utils::declare_and_get_parameter<bool>(
    shared_from_this(), "arm_planning_config.override",
    move_group_types_.arm.override, false);

  get_parameters<std::string>(
    "arm_planning_config."+move_group_types_.arm.planner_id,
    move_group_types_.arm.planner_params);

  arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_,
    move_group_types_.arm.move_group);

  arm_move_group_->setPlanningPipelineId(move_group_types_.arm.planning_pipeline);
  arm_move_group_->setPlannerId(move_group_types_.arm.planner_id);
  arm_move_group_->setPlanningTime(move_group_types_.arm.planning_time);
  arm_move_group_->setReplanAttempts(move_group_types_.arm.planning_attempts);
  arm_move_group_->setMaxVelocityScalingFactor(move_group_types_.arm.vel_acc_scaling_factors[0]);
  arm_move_group_->setMaxAccelerationScalingFactor(move_group_types_.arm.vel_acc_scaling_factors[1]);

  arm_move_group_->setPlannerParams(
    move_group_types_.arm.planner_id,
    move_group_types_.arm.move_group,
    move_group_types_.arm.planner_params,
    move_group_types_.arm.override);

  moveit_visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
    node_, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, arm_move_group_->getRobotModel());
}

void MoveGroupServer::gripper_move_group_init()
{
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_from_this(), "gripper_planning_config.move_group",
    move_group_types_.gripper.move_group, "arm");
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::vector<double>>(
    shared_from_this(), "gripper_planning_config.vel_acc_scaling_factors",
    move_group_types_.gripper.vel_acc_scaling_factors, std::vector<double>({0.6, 0.6}));
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_from_this(), "gripper_planning_config.planning_pipeline",
    move_group_types_.gripper.planning_pipeline, "ompl");
  sh_utils::ros2_node_utils::declare_and_get_parameter<std::string>(
    shared_from_this(), "gripper_planning_config.planner_id",
    move_group_types_.gripper.planner_id, "RRTConnect");
  sh_utils::ros2_node_utils::declare_and_get_parameter<double>(
    shared_from_this(), "gripper_planning_config.planning_time",
    move_group_types_.gripper.planning_time, 5.0);
  sh_utils::ros2_node_utils::declare_and_get_parameter<int>(
    shared_from_this(), "gripper_planning_config.planning_attempts",
    move_group_types_.gripper.planning_attempts, 5);
  sh_utils::ros2_node_utils::declare_and_get_parameter<bool>(
    shared_from_this(), "gripper_planning_config.override",
    move_group_types_.gripper.override, false);

  get_parameters<std::string>(
    "gripper_planning_config."+move_group_types_.gripper.planner_id,
    move_group_types_.gripper.planner_params);

  gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_,
    move_group_types_.gripper.move_group);

  gripper_move_group_->setPlanningPipelineId(move_group_types_.gripper.planning_pipeline);
  gripper_move_group_->setPlannerId(move_group_types_.gripper.planner_id);
  gripper_move_group_->setPlanningTime(move_group_types_.gripper.planning_time);
  gripper_move_group_->setReplanAttempts(move_group_types_.gripper.planning_attempts);
  gripper_move_group_->setMaxVelocityScalingFactor(move_group_types_.gripper.vel_acc_scaling_factors[0]);
  gripper_move_group_->setMaxAccelerationScalingFactor(move_group_types_.gripper.vel_acc_scaling_factors[1]);

  gripper_move_group_->setPlannerParams(
    move_group_types_.gripper.planner_id,
    move_group_types_.gripper.move_group,
    move_group_types_.gripper.planner_params,
    move_group_types_.gripper.override);
}

void MoveGroupServer::select_target_service_callback(
  const std::shared_ptr<SelectNextTargetSrv::Request> /*request*/,
  std::shared_ptr<SelectNextTargetSrv::Response> response)
{
  // TODO: Implement this part as plugin
  std::vector<std::string> object_names = planning_scene_interface_.getKnownObjectNames();
  auto size = object_names.size();

  if (!size) {
    response->success = false;
    return;
  }

  if (size == 1 && object_names[0] == "ground_plane") {
    response->success = false;
    return;
  }

  response->success = true;
  response->target_name = (object_names[0] != "ground_plane") ? object_names[0] : object_names[1];
}

void MoveGroupServer::setup_move_to_named_target_action()
{
  // REQUEST
  auto goal_request = [this](
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const MoveToNamedTargetAction::Goal> goal)
  {
    goal_named_target_ = goal->named_target;
    goal_group_name_ = goal->group_name;

    use_constraints_ = goal->apply_constraint;
    cartesian_ = goal->cartesian;

    if (!select_move_group(goal_group_name_)) {
      RCLCPP_ERROR(
        get_logger(), "%s is not a valid group name. Check the params yaml config.", goal_group_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!named_target_exists(goal->named_target, goal->group_name)) {
      RCLCPP_ERROR(
        get_logger(), "%s is not a valid target. Check your SRDF file.", goal_named_target_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    constraints_->clear_constraints(selected_move_group_, moveit_visual_tools_);

    attach_ = goal->attach;
    detach_ = goal->detach;
    object_to_attach_detach_ = goal->object_to_attach_detach;
    link_to_attach_detach_ = goal->link_to_attach_detach;

    RCLCPP_INFO(get_logger(), "%s target found. Goal accepted.", goal_named_target_.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  // CANCEL
  auto goal_cancel = [this](
    const std::shared_ptr<GoalHandleMoveToNamedTargetAction> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal.");
    selected_move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  // ACCEPT
  auto goal_accepted = [this](
    const std::shared_ptr<GoalHandleMoveToNamedTargetAction> goal_handle)
  {
    auto execute_in_thread = [this, goal_handle](){
      auto result = std::make_shared<MoveToNamedTargetAction::Result>();

      selected_move_group_->setNamedTarget(goal_named_target_);

      if (attach_) {
        selected_move_group_->attachObject(
          object_to_attach_detach_, link_to_attach_detach_, selected_move_group_->getLinkNames());
      }
      if (detach_) {
        selected_move_group_->detachObject(object_to_attach_detach_);
        planning_scene_interface_.removeCollisionObjects({object_to_attach_detach_});
      }

      // if (selected_move_group_->getName() == "arm") {
      //   geometry_msgs::msg::Pose goal_pose;
      //   get_pose_from_named_target(goal_named_target_, goal_pose);

      //   RCLCPP_INFO(get_logger(), "X: %.3f, Y: %.3f, Z: %.3f.", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);

      //   if (use_constraints_) {
      //     constraints_->apply_constraints(selected_move_group_, moveit_visual_tools_, goal_pose);
      //   }

      //   if (cartesian_) {
      //     plan_and_execute_cartesian(goal_handle, result, goal_pose);
      //     result->success = true;
      //     return;
      //   }
      // }

      auto plan_and_execute_result = plan_and_execute<MoveToNamedTargetAction>(goal_handle, result);

      // TODO: Add cancel condition
      if (!plan_and_execute_result) {
        result->success = false;
        if (goal_handle->is_executing()) {
          goal_handle->abort(result);
        }
        return;
      }

      result->success = true;
      goal_handle->succeed(result);
    };
    std::thread{execute_in_thread}.detach();
  };

  // CREATE ACTION
  move_to_named_target_server_ = rclcpp_action::create_server<MoveToNamedTargetAction>(
    this,
    "move_to_named_target",
    // Goal request
    goal_request,
    // Goal cancel
    goal_cancel,
    // Goal accepted
    goal_accepted);
}

void MoveGroupServer::setup_move_to_object_action()
{
  // REQUEST
  auto goal_request = [this](
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const MoveToObjectAction::Goal> goal)
  {
    goal_object_name_ = goal->object_name;
    goal_group_name_ = goal->group_name;

    use_constraints_ = goal->apply_constraint;
    cartesian_ = goal->cartesian;

    if (!select_move_group(goal_group_name_)) {
      RCLCPP_ERROR(
        get_logger(), "%s is not a valid group name. Check the params yaml config.", goal_group_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!object_exists(goal_object_name_)) {
      RCLCPP_ERROR(
        get_logger(), "%s does not exist in the planning scene.", goal_object_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    constraints_->clear_constraints(selected_move_group_, moveit_visual_tools_);

    RCLCPP_INFO(get_logger(), "%s found. Goal accepted.", goal_object_name_.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  // CANCEL
  auto goal_cancel = [this](
    const std::shared_ptr<GoalHandleMoveToObjectAction> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal.");
    selected_move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  // ACCEPT
  auto goal_accepted = [this](
    const std::shared_ptr<GoalHandleMoveToObjectAction> goal_handle)
  {
    auto execute_in_thread = [this, goal_handle](){
      auto result = std::make_shared<MoveToObjectAction::Result>();

      geometry_msgs::msg::Pose goal_pose =
        planning_scene_interface_.getObjectPoses({goal_object_name_})[goal_object_name_];

      geometry_msgs::msg::Pose pre_grasp_pose = compute_pre_grasp(goal_pose, 0.1);
      selected_move_group_->setPoseTarget(pre_grasp_pose);

      auto plan_and_execute_result = plan_and_execute<MoveToObjectAction>(goal_handle, result);

      if (!plan_and_execute_result) {
        result->success = false;
        if (goal_handle->is_executing()) {
          goal_handle->abort(result);
        }
        return;
      }

      // if (use_constraints_) {
      //   constraints_->apply_constraints(selected_move_group_, moveit_visual_tools_, goal_pose);
      // }

      if (cartesian_) {
        auto cart_result = plan_and_execute_cartesian(goal_handle, result, goal_pose);
        if (!cart_result) {
          result->success = false;
          if (goal_handle->is_executing()) {
            goal_handle->abort(result);
          }
          return;
        }
        // result->success = true;
        // return;
      }

      result->success = true;
      goal_handle->succeed(result);
    };
    std::thread{execute_in_thread}.detach();
  };

  // CREATE ACTION
  move_to_object_server_ = rclcpp_action::create_server<MoveToObjectAction>(
    this,
    "move_to_object",
    // Goal request
    goal_request,
    // Goal cancel
    goal_cancel,
    // Goal accepted
    goal_accepted);
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

bool MoveGroupServer::named_target_exists(
  const std::string& named_target,
  const std::string& move_group)
{
  std::vector<std::string> named_target_list;
  {
    // std::lock_guard<std::mutex> lock(param_mutex_);
    if (move_group == move_group_types_.arm.move_group) {
      named_target_list = arm_move_group_->getNamedTargets();
    } else if (move_group == move_group_types_.gripper.move_group) {
      named_target_list = gripper_move_group_->getNamedTargets();
    } else {
      return false;
    }
  }
  auto result = std::find(named_target_list.begin(), named_target_list.end(), named_target);
  if (result != named_target_list.end()) {
    return true;
  }
  return false;
}

void MoveGroupServer::get_pose_from_named_target(
  const std::string& named_target,
  geometry_msgs::msg::Pose& goal_pose)
{
  auto named_joint_values = selected_move_group_->getNamedTargetValues(goal_named_target_);

  moveit::core::RobotState goal_state(selected_move_group_->getRobotModel());
  goal_state.setVariablePositions(named_joint_values);
  goal_state.update();  // propagates FK

  const Eigen::Isometry3d& eef_pose =
    goal_state.getGlobalLinkTransform(selected_move_group_->getEndEffectorLink());

  goal_pose = tf2::toMsg(eef_pose);
}

bool MoveGroupServer::select_move_group(const std::string& move_group)
{
  if (move_group == move_group_types_.arm.move_group) {
    selected_move_group_ = arm_move_group_;
    return true;
  } else if (move_group == move_group_types_.gripper.move_group) {
    selected_move_group_ = gripper_move_group_;
    return true;
  } else {
    return false;
  }
}

template <class MoveAction>
bool MoveGroupServer::plan_and_execute(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveAction>> goal_handle,
  std::shared_ptr<typename MoveAction::Result>& result)
{
  auto feedback = std::make_shared<typename MoveAction::Feedback>();

  moveit::planning_interface::MoveGroupInterface::Plan group_plan;
  // feedback->phase = FeedBackStatus::PLANNING;
  feedback->message = "Planning started.";
  goal_handle->publish_feedback(feedback);

  auto const ok = static_cast<bool>(selected_move_group_->plan(group_plan));
  auto const [success, plan] = std::make_pair(ok, group_plan);

  if (!success) {
    RCLCPP_ERROR(get_logger(), "Planning failed!.");
    result->message = "Planning goes wrong.";
    return false;
  }

  moveit_visual_tools_->publishTrajectoryLine(
    plan.trajectory,
    selected_move_group_->getRobotModel()->getJointModelGroup("arm"));
  moveit_visual_tools_->trigger();

  // if (goal_handle->is_canceling()) {
  //   goal_handle->canceled(result);
  //   return GoalStatus::CANCELED;
  // }

  // feedback->phase = FeedBackStatus::EXECUTING;
  feedback->message = "Path found. Execution started.";
  goal_handle->publish_feedback(feedback);
  auto execution_result = selected_move_group_->execute(plan);

  if (!execution_result) {
    // if (goal_handle->is_canceling()) {
    //   result->success = false;
    //   result->message = "Goal canceled.";
    //   goal_handle->canceled(result);
    //   return GoalStatus::CANCELED;
    // }
    RCLCPP_ERROR(get_logger(), "Path execution failed!. Skipped.");
    result->message = "Path execution failed.";
    return false;
    // return GoalStatus::ABORTED;
  }

  // if (goal_handle->is_canceling()) {
  //   goal_handle->canceled(result);
  //   return GoalStatus::CANCELED;
  // }

  result->message = "Path execution completed.";
  RCLCPP_INFO(get_logger(), "Path execution completed.");
  return true;
}

template <class MoveAction>
bool MoveGroupServer::plan_and_execute_cartesian(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveAction>> goal_handle,
  std::shared_ptr<typename MoveAction::Result>& result,
  geometry_msgs::msg::Pose& goal_pose)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(goal_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = selected_move_group_->computeCartesianPath(waypoints, 0.02, trajectory);

  if (fraction < 0.9) {
    return false;
  }

  moveit_visual_tools_->publishTrajectoryLine(
    trajectory,
    selected_move_group_->getRobotModel()->getJointModelGroup("arm"));
  moveit_visual_tools_->trigger();

  RCLCPP_INFO(get_logger(), "Cartesian path: %.3f", fraction);

  auto execution_result = selected_move_group_->execute(trajectory);

  return true;
}


geometry_msgs::msg::Pose MoveGroupServer::compute_pre_grasp(
  const geometry_msgs::msg::Pose& grasp_pose,
  double offset_m)
{
  RCLCPP_INFO(
    get_logger(),
    "Grasp XYZ: [%.3f, %.3f, %.3f]",
    grasp_pose.position.x,
    grasp_pose.position.y,
    grasp_pose.position.z);

  Eigen::Vector3d approach_axis_ee(0.0, 0.0, 1.0);

  Eigen::Quaterniond q(
    grasp_pose.orientation.w,
    grasp_pose.orientation.x,
    grasp_pose.orientation.y,
    grasp_pose.orientation.z);

  Eigen::Vector3d approach_dir_base = q * approach_axis_ee;
  approach_dir_base.normalize();

  geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
  pre_grasp_pose.position.x -= approach_dir_base.x() * offset_m;
  pre_grasp_pose.position.y -= approach_dir_base.y() * offset_m;
  pre_grasp_pose.position.z -= approach_dir_base.z() * offset_m;

  RCLCPP_INFO(
    get_logger(),
    "Pre grasp XYZ: [%.3f, %.3f, %.3f]",
    pre_grasp_pose.position.x,
    pre_grasp_pose.position.y,
    pre_grasp_pose.position.z);

  return pre_grasp_pose;

}

}  // namespace sh_move_group_server