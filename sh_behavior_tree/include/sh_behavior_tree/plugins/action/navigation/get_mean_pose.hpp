#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__GET_MEAN_POSE_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__GET_MEAN_POSE_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace sh_behavior_tree
{

/**
 * @class sh_behavior_tree::GetMeanPose
 * @brief BehaviorTree Action Node that gets the nearest object pose.
 * Objects frame ID must be 'base_link'
 */
class GetMeanPose : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::GetMeanPose class.
   *
   * TF buffer initialization.
   * @param action_name Name of the action node in the BehaviorTree
   * @param config Configuration of the BehaviorTree Node
   */
  explicit GetMeanPose(const std::string& action_name, const BT::NodeConfig& config);

  /**
   * @brief A destructor for sh_behavior_tree::GetMeanPose class.
   */
  ~GetMeanPose() = default;

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
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // ROS 2 transformation handlers
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATION__GET_MEAN_POSE_HPP_