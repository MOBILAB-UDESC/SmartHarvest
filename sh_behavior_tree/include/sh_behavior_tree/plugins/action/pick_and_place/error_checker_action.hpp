#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__ERROR_CHECKER_ACTION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__ERROR_CHECKER_ACTION_HPP_

#include <chrono>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "sh_interfaces/msg/error_codes.hpp"

namespace sh_behavior_tree
{

/**
 * @class sh_behavior_tree::ErrorCheckerAction
 * @brief BehaviorTree Action Node that transforms object poses into a specified target frame.
 */
class ErrorCheckerAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::ErrorCheckerAction class.
   *
   * @param action_name Name of the action node in the BehaviorTree
   * @param config Configuration of the BehaviorTree Node
   */
  explicit ErrorCheckerAction(const std::string& action_name, const BT::NodeConfig& config);

  /**
   * @brief A destructor for sh_behavior_tree::ErrorCheckerAction class.
   */
  ~ErrorCheckerAction() = default;

  /**
   * @brief Creates list of BT ports.
   *
   * @return BT::PortsList List of ports with their types and descriptions.
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Main execution function required by BehaviorTree.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus tick() override;

  // ROS 2 node
  rclcpp::Logger logger_;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__ERROR_CHECKER_ACTION_HPP_