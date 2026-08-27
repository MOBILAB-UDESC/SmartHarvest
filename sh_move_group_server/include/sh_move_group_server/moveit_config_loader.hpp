#ifndef SH_MOVE_GROUP_SERVER__MOVEIT_CONFIG_LOADER_HPP_
#define SH_MOVE_GROUP_SERVER__MOVEIT_CONFIG_LOADER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace sh_move_group_server
{

/**
 * @struct sh_move_group_server::MoveItConfigOptions
 * @brief Options for the runtime import of the MoveIt configuration.
 */
struct MoveItConfigOptions
{
  std::string source_node{"move_group"};
  std::vector<std::string> parameter_prefixes{
    "robot_description_kinematics",
    "robot_description_planning"};
  double timeout{60.0}; // s
};

/**
 * @class sh_move_group_server::MoveItConfigLoader
 * @brief Copy move_group node parameters into a plain rclcpp::Node.
 */
class MoveItConfigLoader
{
public:
  /**
   * @brief A constructor for sh_move_group_server::MoveItConfigLoader class.
   *
   * @param node Node that will be populated and that is used for the queries.
   * @param logger Logger used to report progress.
   */
  MoveItConfigLoader(const rclcpp::Node::SharedPtr & node, const rclcpp::Logger & logger);

  /**
   * @brief Import the MoveIt config into the node.
   *
   * @return true or false.
   */
  bool load();

private:
  /**
   * @brief Block until move_group shows up in the ROS graph.
   *
   * @param node_name Fully name of the source (/move_group) node.
   * @param deadline Absolute time for the method to wait
   * @return true or false.
   */
  bool wait_for_source_node(
    const std::string & node_name,
    const std::chrono::steady_clock::time_point & deadline);

  /**
   * @brief Copy every parameter matching the configured prefixes from the source node.
   *
   * @param deadline Absolute time for the method to wait
   * @return true or false.s
   */
  bool import_parameters(const std::chrono::steady_clock::time_point & deadline);

  void declare_if_needed(const std::string & name, const rclcpp::ParameterValue & value);

  /**
   * @brief Check whether a parameter name belongs to one of the prefixes.
   *
   * @param name Parameter name.
   * @param prefixes Accepted prefixes.
   * @return true or false.
   */
  static bool matches_prefix(const std::string & name, const std::vector<std::string> & prefixes);

  MoveItConfigOptions options_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
};

}  // namespace sh_move_group_server

#endif  // SH_MOVE_GROUP_SERVER__MOVEIT_CONFIG_LOADER_HPP_
