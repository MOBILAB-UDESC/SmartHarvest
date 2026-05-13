#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__GET_CAMERA_DATA_ACTION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__GET_CAMERA_DATA_ACTION_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sh_base_template/bt_syn_subscription_node.hpp"

namespace sh_behavior_tree
{

// Message type aliases for ROS 2 RGB-D messages
using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

/**
 * @class sh_behavior_tree::GetCameraDataAction
 * @brief BehaviorTree Action Node that synchronously acquires RGB-D data.
 */
class GetCameraDataAction : public sh_base_template::BTSyncSubscriptionNode<
  rclcpp_lifecycle::LifecycleNode, ImageMsg, ImageMsg, CameraInfoMsg>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::GetCameraDataAction class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit GetCameraDataAction(const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::GetCameraDataAction class.
   */
  ~GetCameraDataAction();

  /**
   * @brief Creates list of BT ports.
   *
   * @return BT::PortsList List of ports with their type and description.
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Receives synchronized messages from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus onTick(
    const std::shared_ptr<const ImageMsg>& rgb_msg_,
    const std::shared_ptr<const ImageMsg>& depth_msg_,
    const std::shared_ptr<const CameraInfoMsg>& cam_info_msg) override;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__GET_CAMERA_DATA_ACTION_HPP_