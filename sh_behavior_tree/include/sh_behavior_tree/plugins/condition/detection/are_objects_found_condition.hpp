#ifndef SH_BEHAVIOR_TREE__PLUGINS__CONDITION__DETECTION__ARE_OBJECTS_FOUND_CONDITION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__CONDITION__DETECTION__ARE_OBJECTS_FOUND_CONDITION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

#include "sh_interfaces/msg/detected_objects.hpp"

namespace sh_behavior_tree
{

/**
 * @class sh_behavior_tree::AreObjectsFoundCondition
 * @brief BehaviorTree condition node that checks if any object was detected.
 */
class AreObjectsFoundCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::AreObjectsFoundCondition class.
   *
   * @param condition_name Name of the condition node in the BehaviorTree
   * @param config Configuration of the BehaviorTree Node
   */
  explicit AreObjectsFoundCondition(const std::string& condition_name, const BT::NodeConfig& config);

  /**
   * @brief A destructor for sh_behavior_tree::AreObjectsFoundCondition class.
   */
  ~AreObjectsFoundCondition() = default;

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

  rclcpp::Logger logger_;
  sh_interfaces::msg::DetectedObjects detected_objects;
};

}  // namespace sh_behavior_tree


#endif  // SH_BEHAVIOR_TREE__PLUGINS__CONDITION__DETECTION__ARE_OBJECTS_FOUND_CONDITION_HPP_