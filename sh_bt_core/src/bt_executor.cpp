#include "sh_bt_core/bt_executor.hpp"

#include <chrono>

#include "ament_index_cpp/get_package_share_directory.hpp"


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

BTExecutor::BTExecutor(const std::string & node_name)
: rclcpp_lifecycle::LifecycleNode(node_name),
  logger_(rclcpp::get_logger(node_name))
{
  pre_shutdown_handle_ = get_node_options().context()->add_pre_shutdown_callback(
    [this]() {
      RCLCPP_WARN(logger_, "Shutting down; stopping the tree.");
      stop_running_goal();
    });

  declare_parameters();
}

BTExecutor::~BTExecutor()
{
  get_node_options().context()->remove_pre_shutdown_callback(pre_shutdown_handle_);
}

BTExecutor::CallbackReturn BTExecutor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Configuring.");

  tick_period_s_ = get_parameter("tick_period").as_double();
  feedback_period_s_ = get_parameter("feedback_period").as_double();

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

BTExecutor::CallbackReturn BTExecutor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating.");

  groot_publisher_ =
    std::make_unique<BT::Groot2Publisher>(tree_, 5555);

  setup_tick_action();

  RCLCPP_INFO(logger_, "Activated.");
  return CallbackReturn::SUCCESS;
}

BTExecutor::CallbackReturn BTExecutor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating.");
  stop_running_goal();
  tick_action_server_.reset();
  groot_publisher_.reset();
  RCLCPP_INFO(logger_, "Deactivated.");
  return CallbackReturn::SUCCESS;
}

BTExecutor::CallbackReturn BTExecutor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up.");
  return CallbackReturn::SUCCESS;
}

BTExecutor::CallbackReturn BTExecutor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down.");
  return CallbackReturn::SUCCESS;
}

void BTExecutor::declare_parameters()
{
  declare_parameter("tree_xml_path", rclcpp::ParameterValue(""));
  declare_parameter("tick_period", rclcpp::ParameterValue(0.01));
  declare_parameter("feedback_period", rclcpp::ParameterValue(0.2));
  declare_parameter("enable_groot", rclcpp::ParameterValue(true));

  declare_parameter("default_sync_timeout", rclcpp::ParameterValue(0.25));

  declare_parameter("default_sub_timeout", rclcpp::ParameterValue(0.15));

  declare_parameter("default_service_response_timeout", rclcpp::ParameterValue(1.0));
  declare_parameter("wait_for_service_timeout", rclcpp::ParameterValue(2.0));

  declare_parameter("default_action_response_timeout", rclcpp::ParameterValue(0.0));
  declare_parameter("wait_for_action_timeout", rclcpp::ParameterValue(2.0));
  declare_parameter("default_cancel_timeout", rclcpp::ParameterValue(2.0));

  declare_parameter(
    "camera_default_topics",
    rclcpp::ParameterValue(
      std::vector<std::string>({
        "camera_0/rgb/image",
        "camera_0/depth/image",
        "camera_0/rgb/camera_info"})));
  declare_parameter("detection_service_name", rclcpp::ParameterValue("get_detections"));
  declare_parameter("transformation_timeout", rclcpp::ParameterValue(0.50));

  declare_parameter("bt_plugin_ids", rclcpp::ParameterValue(std::vector<std::string>({})));
}

BT::Blackboard::Ptr BTExecutor::setup_blackboard()
{
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();

  blackboard->set("root_node", weak_from_this());

  blackboard->set("default_cancel_timeout", get_parameter("default_cancel_timeout").as_double());

  blackboard->set("default_sync_timeout", get_parameter("default_sync_timeout").as_double());

  blackboard->set("wait_for_service_timeout", get_parameter("wait_for_service_timeout").as_double());
  blackboard->set(
    "default_service_response_timeout",
    get_parameter("default_service_response_timeout").as_double());

  blackboard->set(
    "default_action_response_timeout",
    get_parameter("default_action_response_timeout").as_double());
  blackboard->set("wait_for_action_timeout", get_parameter("wait_for_action_timeout").as_double());

  blackboard->set("camera_default_topics", get_parameter("camera_default_topics").as_string_array());
  blackboard->set("transformation_timeout", get_parameter("transformation_timeout").as_double());

  blackboard->set("default_sub_timeout", get_parameter("default_sub_timeout").as_double());

  return blackboard;
}

void BTExecutor::setup_tick_action()
{
  // REQUEST
  auto goal_request = [this](
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const TickAction::Goal> /*goal*/)
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
    RCLCPP_WARN(logger_, "Cancel requested.");
    cancel_token_->store(true);

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
    cancel_token_->store(false);

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

void BTExecutor::execute_tick(const std::shared_ptr<GoalHandleTickAction> & goal_handle)
{
  auto active_node_tracker = std::make_unique<ActiveNodeTracker>(tree_.rootNode());
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<TickAction::Result>();
  auto feedback = std::make_shared<TickAction::Feedback>();

  RCLCPP_INFO(logger_, "Ticking the tree.");

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  bool cancelled = false;
  uint32_t ticks = 0;

  const auto started = std::chrono::steady_clock::now();
  const auto tick_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(tick_period_s_));
  const auto feedback_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(feedback_period_s_));

  std::string last_active_node;
  auto last_feedback = std::chrono::steady_clock::time_point::min();

  while (!cancelled) {
    if (goal_handle->is_canceling() || cancel_token_->load() || !rclcpp::ok()) {
      cancelled = true;
      tree_.haltTree();
      break;
    }

    status = tree_.tickOnce();
    ++ticks;

    const auto now = std::chrono::steady_clock::now();
    auto active_node = active_node_tracker->active_node();

    if (active_node != last_active_node || now - last_feedback >= feedback_period) {
      feedback->status = static_cast<uint8_t>(status);
      feedback->ticks = ticks;
      feedback->active_node = active_node;
      feedback->elapsed_time = std::chrono::duration<double>(now - started).count();
      goal_handle->publish_feedback(feedback);
      last_active_node = active_node;
      last_feedback = now;
    }

    if (status != BT::NodeStatus::RUNNING) {
      break;
    }

    std::this_thread::sleep_for(tick_period);
  }

  result->status = static_cast<uint8_t>(status);
  result->ticks = ticks;
  result->elapsed_time =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

  if (cancelled) {
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
  cancel_token_->store(true);

  if (tick_thread_.joinable()) {
    RCLCPP_INFO(logger_, "Stop requested; waiting for the tick to unwind.");
    tick_thread_.join();
    RCLCPP_INFO(logger_, "Tick stopped.");
  }

  cancel_token_->store(false);
  goal_running_.store(false);
}

}  // namespace sh_bt_core
