#include "rclcpp/rclcpp.hpp"

#include "sh_planning_scene_handler/planning_scene_handler.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto scene_handler_node = std::make_shared<sh_planning_scene_handler::PlanningSceneHandler>
    ("sh_planning_scene_handler");

  rclcpp::spin(scene_handler_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}