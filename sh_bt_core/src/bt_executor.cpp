#include "sh_bt_core/bt_executor.hpp"

namespace sh_bt_core
{

ActiveNodeTracker::ActiveNodeTracker(BT::TreeNode * root_node)
: BT::StatusChangeLogger(root_node)
{}

void ActiveNodeTracker::callback(
  BT::Duration /*timestamp*/,
  const BT::TreeNode & node,
  BT::NodeStatus /*prev_status*/,
  BT::NodeStatus status)
{
  if (status != BT::NodeStatus::RUNNING) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  active_node_ = node.name();
}

void ActiveNodeTracker::flush()
{}

std::string ActiveNodeTracker::active_node()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_node_;
}

BTExecutor::BTExecutor(const std::string & node_name):
  rclcpp_lifecycle::LifecycleNode(node_name),
  logger_(rclcpp::get_logger(node_name))
{
  declare_parameters();
}

CallbackReturn BTExecutor::on_configure(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(logger_, "Configuring.");

  BT::Blackboard::Ptr blackboard = setup_blackboard();

  // Add default and custom BT nodes to the factory.
  // Custom nodes are passed in config.yaml.
  BT::BehaviorTreeFactory factory;
  register_default_nodes(factory);

  std::vector<std::string> plugins_ids = get_parameter("bt_plugin_ids").as_string_array();
  register_from_plugin(factory, plugins_ids);

  std::string tree_xml = get_parameter("tree_xml_path").as_string();
  if (tree_xml == "") {
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("sh_behavior_tree");
    tree_xml = pkg_share_dir + "/trees/sh_detection_stage_tree.xml";
  }

  factory.registerBehaviorTreeFromFile(tree_xml);
  tree_ = factory.createTree("MainTree", blackboard);

  RCLCPP_INFO(logger_, "Configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BTExecutor::on_activate(const rclcpp_lifecycle::State& state)
{
  (void) state;
  RCLCPP_INFO(logger_, "Activating.");

  groot_publisher_ =
    std::make_unique<BT::Groot2Publisher>(tree_, 5555);

  tick_service_ = this->create_service<sh_interfaces::srv::TickTree>("tick",
    std::bind(
      &BTExecutor::tick_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  setup_tick_action();

  RCLCPP_INFO(logger_, "Activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BTExecutor::on_deactivate(const rclcpp_lifecycle::State& state)
{
  (void) state;
  RCLCPP_INFO(logger_, "Deactivating.");
  tick_service_.reset();
  stop_running_goal();
  RCLCPP_INFO(logger_, "Deactivated.");
  return CallbackReturn::SUCCESS;
}

void BTExecutor::declare_parameters()
{
  declare_parameter("tree_xml_path", rclcpp::ParameterValue(""));
  declare_parameter("enable_groot", rclcpp::ParameterValue(true));

  declare_parameter("default_sync_timeout", rclcpp::ParameterValue(0.25));

  declare_parameter("default_sub_timeout", rclcpp::ParameterValue(0.15));

  declare_parameter("default_service_response_timeout", rclcpp::ParameterValue(1.0));
  declare_parameter("wait_for_service_timeout", rclcpp::ParameterValue(2.0));

  declare_parameter("default_action_response_timeout", rclcpp::ParameterValue(0.0));
  declare_parameter("wait_for_action_timeout", rclcpp::ParameterValue(2.0));

  declare_parameter("camera_default_topics", rclcpp::ParameterValue(
    std::vector<std::string>({
      "camera_0/rgb/image",
      "camera_0/depth/image",
      "camera_0/rgb/camera_info"})));
  declare_parameter("detection_service_name", rclcpp::ParameterValue("get_detections"));
  declare_parameter("camera_to_end_effector_transform",
    rclcpp::ParameterValue(std::vector<double>({0.0, 0.0, 0.0})));
  declare_parameter("transformation_timeout", rclcpp::ParameterValue(0.50));
  declare_parameter("clear_scene_service_name", rclcpp::ParameterValue("clear_scene"));
  declare_parameter("update_scene_service_name", rclcpp::ParameterValue("update_from_pose_scene"));
  declare_parameter("move_to_named_target_action_name", rclcpp::ParameterValue("move_to_named_target"));
  declare_parameter("move_to_pose_action_name", rclcpp::ParameterValue("move_to_pose"));
  declare_parameter("move_to_object_action_name", rclcpp::ParameterValue("move_to_object"));
  declare_parameter("object_name_prefix", rclcpp::ParameterValue("Apple"));

  declare_parameter("bt_plugin_ids", rclcpp::ParameterValue(std::vector<std::string>({})));
  declare_parameter("auto_mode", rclcpp::ParameterValue(false));
}

BT::Blackboard::Ptr BTExecutor::setup_blackboard()
{
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();

  blackboard->set("root_node", weak_from_this());

  blackboard->set("default_sync_timeout", get_parameter("default_sync_timeout").as_double());

  blackboard->set("wait_for_service_timeout", get_parameter("wait_for_service_timeout").as_double());
  blackboard->set("default_service_response_timeout",
    get_parameter("default_service_response_timeout").as_double());
  blackboard->set("default_action_response_timeout",
    get_parameter("default_action_response_timeout").as_double());
  blackboard->set("wait_for_action_timeout", get_parameter("wait_for_action_timeout").as_double());

  blackboard->set("camera_default_topics", get_parameter("camera_default_topics").as_string_array());
  blackboard->set("detection_service_name", get_parameter("detection_service_name").as_string());
  blackboard->set("camera_to_end_effector_transform",
    get_parameter("camera_to_end_effector_transform").as_double_array());
  blackboard->set("transformation_timeout", get_parameter("transformation_timeout").as_double());
  blackboard->set("clear_scene_service_name", get_parameter("clear_scene_service_name").as_string());
  blackboard->set("update_scene_service_name", get_parameter("update_scene_service_name").as_string());
  blackboard->set("move_to_named_target_action_name", get_parameter("move_to_named_target_action_name").as_string());
  blackboard->set("move_to_pose_action_name", get_parameter("move_to_pose_action_name").as_string());
  blackboard->set("move_to_object_action_name", get_parameter("move_to_object_action_name").as_string());
  blackboard->set("object_name_prefix", get_parameter("object_name_prefix").as_string());

  blackboard->set("default_sub_timeout", get_parameter("default_sub_timeout").as_double());

  return blackboard;
}

void BTExecutor::setup_tick_action()
{
  // REQUEST
  auto goal_request = [this](
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const TickAction::Goal> goal)
  {
    if (goal_running_.load()) {
      RCLCPP_WARN(logger_, "The tree is already being ticked. Goal rejected.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(logger_, "Tick goal accepted.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  // CANCEL
  auto goal_cancel = [this](
    const std::shared_ptr<GoalHandleTickAction> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal.");

    return rclcpp_action::CancelResponse::ACCEPT;
  };

  // ACCEPT
  auto goal_accepted = [this](
    const std::shared_ptr<GoalHandleTickAction> goal_handle)
  {
    if (tick_thread_.joinable()) {
      tick_thread_.join();
    }

    goal_running_.store(true);

    tick_thread_ = std::thread(
      [this, goal_handle]() {
        execute_tick(goal_handle);
        goal_running_.store(false);
    });
  };

  tick_action_server_ = rclcpp_action::create_server<TickAction>(
    this,
    "tick",
    // Goal request
    goal_request,
    // Goal cancel
    goal_cancel,
    // Goal accepted
    goal_accepted);

  RCLCPP_INFO(logger_, "'tick' action server ready.");
}

void BTExecutor::execute_tick(const std::shared_ptr<GoalHandleTickAction> goal_handle)
{
  auto active_node_tracker = std::make_unique<ActiveNodeTracker>(tree_.rootNode());
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<TickAction::Result>();
  auto feedback = std::make_shared<TickAction::Feedback>();

  RCLCPP_INFO(logger_, "Ticking the tree.");

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  bool cancelled = false;
  uint32_t ticks = 0;

  while (!cancelled) {
    if (goal_handle->is_canceling()) {
      cancelled = true;
    }

    status = tree_.tickOnce();
    ++ticks;

    feedback->status = static_cast<uint8_t>(status);
    feedback->ticks = ticks;
    feedback->active_node = active_node_tracker->active_node();
    goal_handle->publish_feedback(feedback);

    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
  }

  result->status = static_cast<uint8_t>(status);
  result->ticks = ticks;

  if (cancelled) {
    tree_.haltTree();
    result->success = false;
    result->message = "Task cancelled.";
    RCLCPP_WARN(logger_, "%s", result->message.c_str());
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    return;
  }

  switch (status) {
    case BT::NodeStatus::SUCCESS:
      result->success = true;
      result->message = "Task successfully completed.";
      RCLCPP_INFO(logger_, "%s", result->message.c_str());
      goal_handle->succeed(result);
      return;

    default:
      result->success = false;
      result->message = "Task failed.";
      RCLCPP_ERROR(logger_, "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
  }
}

void BTExecutor::stop_running_goal()
{
  if (tick_thread_.joinable()) {
    tick_thread_.join();
  }

  goal_running_.store(false);
}

void BTExecutor::tick_callback(
  const std::shared_ptr<sh_interfaces::srv::TickTree::Request> request,
  std::shared_ptr<sh_interfaces::srv::TickTree::Response> response)
{
  (void) request;
  RCLCPP_INFO(logger_, "Ticking the tree.");

  BT::NodeStatus tick_result = tree_.tickWhileRunning();
  if (tick_result == BT::NodeStatus::SUCCESS) {
    response->success = true;
    response->message = "Task successfully completed.";
    RCLCPP_INFO(logger_, "%s", response->message.c_str());
  } else if (tick_result == BT::NodeStatus::SKIPPED) {
    response->success = false;
    response->message = "Task skipped.";
    RCLCPP_WARN(logger_, "%s", response->message.c_str());
  } else {
    response->success = false;
    response->message = "Task failed.";
    RCLCPP_ERROR(logger_, "%s", response->message.c_str());
  }
}

}  // namespace sh_bt_core
