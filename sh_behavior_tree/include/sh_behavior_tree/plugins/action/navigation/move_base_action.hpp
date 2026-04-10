#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__MOVE_BASE_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__MOVE_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "sh_bt_base_template/bt_action_node.hpp"

namespace sh_behavior_tree
{

using nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

/**
 * @class sh_behavior_tree::MoveBaseAction
 * @brief BehaviorTree Action Node that moves the base to a given pose.
 */
class MoveBaseAction :
  public sh_bt_base_template::BTActionNode<rclcpp_lifecycle::LifecycleNode, NavigateToPose>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::MoveBaseAction class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit MoveBaseAction(
    const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::MoveBaseAction class.
   */
  ~MoveBaseAction();

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
  bool update_goal(std::shared_ptr<NavigateToPose::Goal>& goal_msg) override;

  void on_feedback(const std::shared_ptr<const NavigateToPose::Feedback> feedback) override;

  /**
   * @brief Receives response from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus on_success(
    const GoalHandleNavigateToPose::WrappedResult& result) override;

  /**
   * @brief Receives response from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  void on_failure(
    const GoalHandleNavigateToPose::WrappedResult& result) override;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__MOVE_BASE_HPP_