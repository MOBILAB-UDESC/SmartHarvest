#include "sh_behavior_tree/plugins/action/common/measure_time.hpp"

#include <chrono>

namespace sh_behavior_tree
{

MeasureTime::MeasureTime(
  const std::string& action_name, const BT::NodeConfig& config):
  BT::SyncActionNode(action_name, config), logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Node created.");
}

BT::PortsList MeasureTime::providedPorts() {
  return{
    BT::InputPort<bool>("start", "Wheter to start or stop the timer."),
    BT::BidirectionalPort<std::chrono::_V2::steady_clock::time_point>("start_time", "Start time.")
  };
}

BT::NodeStatus MeasureTime::tick()
{
  bool start;
  if (!getInput("start", start)) {
    start = false;
  }

  auto time_now = std::chrono::steady_clock::now();
  if (start) {
    setOutput("start_time", time_now);
    return BT::NodeStatus::SUCCESS;
  }

  std::chrono::_V2::steady_clock::time_point start_time;
  if (!getInput("start_time", start_time)) {
    RCLCPP_ERROR(logger_, "Missing 'start_time' port value.");
    return BT::NodeStatus::FAILURE;
  }

  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - start_time).count();
  RCLCPP_INFO(logger_, "\033[1;32mElapsed time: %ld.\033[0m", elapsed_time);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::MeasureTime>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::MeasureTime>(
    "MeasureTime", builder);
}