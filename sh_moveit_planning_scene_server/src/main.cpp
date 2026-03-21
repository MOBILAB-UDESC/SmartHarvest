#include "rclcpp/rclcpp.hpp"

#include "sh_moveit_planning_scene_server/planning_scene_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto scene_server_node = std::make_shared<sh_moveit_planning_scene_server::PlanningSceneServer>
    ("sh_planning_scene_server");

  rclcpp::spin(scene_server_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}