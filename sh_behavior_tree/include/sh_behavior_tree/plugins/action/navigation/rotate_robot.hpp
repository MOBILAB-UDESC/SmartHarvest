#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__ROTATE_ROBOT_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__ROTATE_ROBOT_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_bt_base_template/bt_subscription_node.hpp"

namespace sh_behavior_tree
{

/**
 * @class sh_behavior_tree::RotateRobot
 * @brief BehaviorTree Action Node that controls the rotation of the robot.
 */
class RotateRobot :
  public sh_bt_base_template::BTSubscriptionNode<rclcpp_lifecycle::LifecycleNode, nav_msgs::msg::Odometry>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::RotateRobot class.
   *
   * Creates velocity publisher and validates required input ports.
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit RotateRobot(const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::RotateRobot class.
   */
  ~RotateRobot() = default;

  /**
   * @brief Creates list of BT ports.
   *
   * @return BT::PortsList List of ports with their type and description.
   */
  static BT::PortsList providedPorts();

private:
  /** Callback invoked by tick() when no message arrives on time.
   *
   * Fails after three consecutive timeouts.
   * @return BT::NodeStatus SUCCESS, FAILURE or RUNNING.
   */
  BT::NodeStatus on_timeout() override;

  /** Callback invoked by tick() when a new message is available.
   *
   * Implementation of a simple proportional controller.
   * @param last_msg Latest message received.
   *
   * @return BT::NodeStatus SUCCESS, FAILURE or RUNNING.
   */
  BT::NodeStatus on_tick(const std::shared_ptr<nav_msgs::msg::Odometry>& last_msg) override;

  // Rotation control settings
  double max_w_;
  double angle_rad_;
  double angle_tolerance_;
  double kp_;

  bool goal_initialized_{false};
  double goal_yaw_;

  int timeout_count_{0};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__ROTATE_ROBOT_HPP_
