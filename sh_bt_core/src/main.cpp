#include "sh_bt_core/bt_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto recoveries_node = std::make_shared<sh_bt_core::BTExecutor>("sh_bt_core");

  rclcpp::spin(recoveries_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}