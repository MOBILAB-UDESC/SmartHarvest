#include "rclcpp/rclcpp.hpp"

#include "sh_move_group_server/move_group_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto movegroup_server_node =
    std::make_shared<sh_move_group_server::MoveGroupServer>("sh_move_group_server");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(movegroup_server_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}