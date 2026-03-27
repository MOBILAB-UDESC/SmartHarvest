#include "rclcpp/rclcpp.hpp"

#include "sh_move_group_server/move_group_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto movegroup_server_node = std::make_shared<sh_move_group_server::MoveGroupServer>
    ("sh_move_group_server", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(movegroup_server_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}