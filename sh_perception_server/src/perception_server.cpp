#include "sh_perception_server/perception_server.hpp"

#include <functional>

#include "pluginlib/exceptions.hpp"

namespace sh_perception_server
{

PerceptionServer::PerceptionServer(const std::string & node_name) :
  rclcpp_lifecycle::LifecycleNode(node_name),
  pipeline_loader_("sh_base_template", "sh_base_template::PerceptionPipelineBase")
{
  RCLCPP_INFO(get_logger(), "Initializing");

  declare_parameter<std::string>("pipeline.plugin", "sh_rgbd_perception_pipeline::RgbdPerceptionPipeline");
}

PerceptionServer::~PerceptionServer()
{
  pipeline_->cleanup();
  pipeline_.reset();
  service_.reset();
}

CallbackReturn PerceptionServer::on_configure(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(get_logger() , "Configuring.");

  get_parameter<std::string>("pipeline.plugin", pipeline_plugin_name_);

  try {
    RCLCPP_INFO(get_logger(), "\033[1;32m[%s] loaded.\033[0m", pipeline_plugin_name_.c_str());
    pipeline_ = pipeline_loader_.createUniqueInstance(pipeline_plugin_name_);
  } catch (const pluginlib::PluginlibException& e) {
    RCLCPP_ERROR(get_logger(), "Error loading pipeline plugin: %s", e.what());
    on_cleanup(state);
    return CallbackReturn::FAILURE;
  }

  if (!pipeline_->configure(weak_from_this())) {
    on_cleanup(state);
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PerceptionServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger() , "Activating.");

  pipeline_->activate();

  service_ = this->create_service<sh_interfaces::srv::RunPerception>(
    "run_perception",
    std::bind(
      &PerceptionServer::trigger_perception_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(get_logger() , "Ready for requests.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PerceptionServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger() , "Deactivating.");
  pipeline_->deactivate();
  service_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PerceptionServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger() , "Cleaning up.");
  pipeline_->cleanup();
  pipeline_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PerceptionServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger() , "Shutting down.");
  return CallbackReturn::SUCCESS;
}

void PerceptionServer::trigger_perception_callback(
  const std::shared_ptr<sh_interfaces::srv::RunPerception::Request> request,
  std::shared_ptr<sh_interfaces::srv::RunPerception::Response> response)
{
  RCLCPP_INFO(get_logger() , "Request received.");

  if (!pipeline_->process(request->input, response->scene))
  {
    response->success = false;
    return;
  }

  response->success = true;
}


}  // namespace sh_perception_server