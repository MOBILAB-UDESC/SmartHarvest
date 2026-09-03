#include "rclcpp/rclcpp.hpp"

#include "sh_bt_core/bt_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sh_bt_core::BTExecutor>("sh_bt_core");

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}