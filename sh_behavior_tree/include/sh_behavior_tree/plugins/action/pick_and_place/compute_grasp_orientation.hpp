#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__COMPUTE_GRASP_ORIENTATION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__COMPUTE_GRASP_ORIENTATION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sh_interfaces/msg/detected_objects.hpp"

namespace sh_behavior_tree
{

/**
 * @class sh_behavior_tree::ComputeGraspOrientation
 * @brief BehaviorTree Action Node that applies a fixed camera-to-end-effector Euler offset
 * to each detected object grasp orientation and updates quaternion representation.
 */
class ComputeGraspOrientation : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::ComputeGraspOrientation class.
   *
   * @param action_name Name of the action node in the BehaviorTree
   * @param config Configuration of the BehaviorTree Node
   */
  explicit ComputeGraspOrientation(const std::string& action_name, const BT::NodeConfig& config);

  /**
   * @brief A destructor for sh_behavior_tree::ComputeGraspOrientation class.
   */
  ~ComputeGraspOrientation() = default;

  /**
   * @brief Creates list of BT ports.
   *
   * @return BT::PortsList List of ports with their types and descriptions.
   */
  static BT::PortsList providedPorts()
  {
    return{
      BT::BidirectionalPort<sh_interfaces::msg::DetectedObjects>("objects", "List of detected objects"),
    };
  }

private:
  /**
   * @brief Main execution function required by BehaviorTree.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus tick() override;

  // ROS 2 node
  rclcpp::Logger logger_;

  // Port variables
  std::vector<double> camera_to_end_effector_transform_;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__COMPUTE_GRASP_ORIENTATION_HPP_