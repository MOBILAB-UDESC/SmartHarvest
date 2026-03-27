#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__GET_CURRENT_OBJECT_ACTION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__GET_CURRENT_OBJECT_ACTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sh_behavior_tree
{

/**
 * @class sh_behavior_tree::GetCurrentObjectAction
 * @brief BehaviorTree Action Node that transforms object poses into a specified target frame.
 */
class GetCurrentObjectAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::GetCurrentObjectAction class.
   *
   * @param action_name Name of the action node in the BehaviorTree
   * @param config Configuration of the BehaviorTree Node
   */
  explicit GetCurrentObjectAction(const std::string& action_name, const BT::NodeConfig& config);

  /**
   * @brief A destructor for sh_behavior_tree::GetCurrentObjectAction class.
   */
  ~GetCurrentObjectAction() = default;

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

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__GET_CURRENT_OBJECT_ACTION_HPP_