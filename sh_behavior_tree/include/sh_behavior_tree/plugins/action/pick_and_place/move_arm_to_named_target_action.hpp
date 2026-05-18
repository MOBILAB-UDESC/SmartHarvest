#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__MOVE_ARM_TO_NAMED_TARGET_ACTION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__MOVE_ARM_TO_NAMED_TARGET_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/bt_action_node.hpp"
#include "sh_interfaces/action/move_to_named_target.hpp"

namespace sh_behavior_tree
{

using MoveToNamedTargetAction = sh_interfaces::action::MoveToNamedTarget;
using GoalHandleMoveToNamedTargetAction = rclcpp_action::ClientGoalHandle<MoveToNamedTargetAction>;

/**
 * @class sh_behavior_tree::MoveArmToNamedTargetAction
 * @brief BehaviorTree Action Node that moves the arm to a named target pose.
 */
class MoveArmToNamedTargetAction :
  public sh_base_template::BTActionNode<rclcpp_lifecycle::LifecycleNode, MoveToNamedTargetAction>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::MoveArmToNamedTargetAction class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit MoveArmToNamedTargetAction(
    const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::MoveArmToNamedTargetAction class.
   */
  ~MoveArmToNamedTargetAction();

  /**
   * @brief Creates list of BT ports.
   *
   * @return BT::PortsList List of ports with their type and description.
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Method invoked by tick() to update the goal msg.
   *
   * @param goal_msg Action goal to send.
   */
  bool update_goal(std::shared_ptr<typename MoveToNamedTargetAction::Goal>& goal_msg) override;

  void on_feedback(const std::shared_ptr<const MoveToNamedTargetAction::Feedback> feedback) override;

  /**
   * @brief Receives response from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus on_success(
    const typename rclcpp_action::ClientGoalHandle<MoveToNamedTargetAction>::WrappedResult& result) override;

  /**
   * @brief Receives response from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  void on_failure(
    const typename rclcpp_action::ClientGoalHandle<MoveToNamedTargetAction>::WrappedResult& result) override;

  void on_timeout()
  {
    setOutput("error_code", sh_interfaces::msg::ErrorCodes::TIMEOUT);
  }
};

}  // namespace sh_behavior_tree


#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__MOVE_ARM_TO_NAMED_TARGET_ACTION_HPP_