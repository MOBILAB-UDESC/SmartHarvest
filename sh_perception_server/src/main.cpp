#include "rclcpp/rclcpp.hpp"

#include "sh_perception_server/perception_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto perception_node = std::make_shared<sh_perception_server::PerceptionServer>("sh_perception_server");

  rclcpp::spin(perception_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}