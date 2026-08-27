#include "sh_move_group_server/moveit_config_loader.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>

#include "std_msgs/msg/string.hpp"

namespace sh_move_group_server
{

using namespace std::chrono_literals;

/// Remaining time before the deadline
std::chrono::milliseconds remaining(const std::chrono::steady_clock::time_point & deadline)
{
  auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
    deadline - std::chrono::steady_clock::now());
  return (left.count() > 0) ? left : 0ms;
}

/// Parameters are fetched in batches to keep the service responses small.
constexpr std::size_t kParameterBatchSize = 40;

MoveItConfigLoader::MoveItConfigLoader(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Logger & logger) :
    node_(node),
    logger_(logger)
{}

bool MoveItConfigLoader::load()
{
  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(options_.timeout * 1000.0));

  RCLCPP_INFO(logger_, "Importing the MoveIt configuration from [%s].", options_.source_node.c_str());

  if (!wait_for_source_node("/" + options_.source_node, deadline)) {
    return false;
  }

  return import_parameters(deadline);
}

bool MoveItConfigLoader::wait_for_source_node(
  const std::string & node_name,
  const std::chrono::steady_clock::time_point & deadline)
{
  while (rclcpp::ok() && remaining(deadline) > 0ms) {
    const auto names = node_->get_node_names();
    if (std::find(names.begin(), names.end(), node_name) != names.end()) {
      return true;
    }
    RCLCPP_INFO_THROTTLE(
      logger_, *node_->get_clock(), 5000, "Waiting for [%s] to come up.", node_name.c_str());
    std::this_thread::sleep_for(200ms);
  }

  RCLCPP_ERROR(logger_, "[%s] never showed up in the ROS graph.", node_name.c_str());
  return false;
}

bool MoveItConfigLoader::import_parameters(
  const std::chrono::steady_clock::time_point & deadline)
{
  // Check if service is available
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node_, options_.source_node);
  if (!client->wait_for_service(remaining(deadline))) {
    RCLCPP_ERROR(
      logger_, "Parameter services of [%s] are not available.", options_.source_node.c_str());
    return false;
  }

  // Get full list of parameters
  auto list_future = client->list_parameters({}, 0);
  if (list_future.wait_for(remaining(deadline)) != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "Timed out listing the parameters of [%s].", options_.source_node.c_str());
    return false;
  }

  // Filter parameters
  std::vector<std::string> wanted_params;
  for (const auto & name : list_future.get().names) {
    if (matches_prefix(name, options_.parameter_prefixes)) {
      wanted_params.push_back(name);
    }
  }

  if (wanted_params.empty()) {
    RCLCPP_ERROR(
      logger_,
      "[%s] does not expose any of the requested parameter prefixes.",
      options_.source_node.c_str());
    return false;
  }

  // When a parameter in the wanted_params list has no value, the service returns an empty response.
  // This causes the whole batch to be dropped.
  // Splitting the list ensures that parameters with valid values can still be retrieved.
  // auto batch = std::vector<std::string>(
  //   wanted_params.begin(),
  //   wanted_params.begin() + 40);

  // auto get_future = client->get_parameters(batch);
  // auto status = get_future.wait_for(remaining(deadline));

  // if (status != std::future_status::ready) {
  //   return false;
  // }

  // const auto values = get_future.get();
  // RCLCPP_INFO(logger_, "requested = %zu, returned = %zu", batch.size(), values.size());

  std::size_t imported = 0;
  bool timed_out = false;
  std::function<void(const std::vector<std::string> &)> fetch =
    [&](const std::vector<std::string> & batch) {
      if (batch.empty() || timed_out) {
        return;
      }

      auto parameters = client->get_parameters(batch);
      if (parameters.wait_for(remaining(deadline)) != std::future_status::ready) {
        timed_out = true;
        return;
      }

      const auto values = parameters.get();
      // When the whole batch has valid values
      if (values.size() == batch.size()) {
        for (const auto & parameter : values) {
          declare_if_needed(parameter.get_name(), parameter.get_parameter_value());
          ++imported;
        }
        return;
      }

      if (batch.size() == 1) {
        RCLCPP_DEBUG(
          logger_, "[%s] has no value on [%s], skipped.",
          batch.front().c_str(), options_.source_node.c_str());
        return;
      }

      const auto middle = batch.begin() + static_cast<long>(batch.size() / 2);
      fetch(std::vector<std::string>(batch.begin(), middle));
      fetch(std::vector<std::string>(middle, batch.end()));
    };

  for (std::size_t i = 0; i < wanted_params.size(); i += kParameterBatchSize) {
    fetch(std::vector<std::string>(
      wanted_params.begin() + static_cast<long>(i),
      wanted_params.begin() + static_cast<long>(std::min(i + kParameterBatchSize, wanted_params.size()))));
  }

  if (timed_out) {
    RCLCPP_ERROR(
      logger_, "Timed out reading the parameters of [%s].", options_.source_node.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger_, "\033[1;32m%zu MoveIt parameters imported from [%s].\033[0m",
    imported, options_.source_node.c_str());

  return true;
}

void MoveItConfigLoader::declare_if_needed(
  const std::string & name,
  const rclcpp::ParameterValue & value)
{
  if (node_->has_parameter(name)) {
    node_->set_parameter(rclcpp::Parameter(name, value));
    return;
  }
  node_->declare_parameter(name, value);
}

bool MoveItConfigLoader::matches_prefix(
  const std::string & name,
  const std::vector<std::string> & prefixes)
{
  return std::any_of(
    prefixes.begin(), prefixes.end(),
    [&name](const std::string & prefix) {
      return name == prefix || name.rfind(prefix + ".", 0) == 0;
    });
}

}  // namespace sh_move_group_server
