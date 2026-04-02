#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__COMMON__TRANSFORM_OBJECT_POSE_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__COMMON__TRANSFORM_OBJECT_POSE_HPP_

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "sh_interfaces/msg/detected_objects.hpp"

namespace sh_behavior_tree
{

/**
 * @class sh_behavior_tree::TransformObjectPose
 * @brief BehaviorTree Action Node that transforms object poses into a specified target frame.
 */
class TransformObjectPose : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::TransformObjectPose class.
   *
   * @param action_name Name of the action node in the BehaviorTree
   * @param config Configuration of the BehaviorTree Node
   */
  explicit TransformObjectPose(const std::string& action_name, const BT::NodeConfig& config);

  /**
   * @brief A destructor for sh_behavior_tree::TransformObjectPose class.
   */
  ~TransformObjectPose() = default;

  /**
   * @brief Creates list of BT ports.
   *
   * @return BT::PortsList List of ports with their types and descriptions.
   */
  static BT::PortsList providedPorts()
  {
    return{
      BT::InputPort<sh_interfaces::msg::DetectedObjects>("objects", "List of detected objects"),
      BT::InputPort<std::string>("transform_frame", "Frame to transform detected object poses into"),
      BT::OutputPort<sh_interfaces::msg::DetectedObjects>(
        "transformed_objects", "List of detected objects with poses transformed into a target frame")
    };
  }

private:
  /**
   * @brief Main execution function required by BehaviorTree.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Transform pose to another frame.
   *
   * @return true or false.
   */
  bool transform_pose(
    const sh_interfaces::msg::DetectedObjects& objects,
    const std::string& transform_frame);

  // ROS 2 node
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_;

  // ROS 2 transformation handlers
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Port variables
  double transformation_timeout_;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__COMMON__TRANSFORM_OBJECT_POSE_HPP_