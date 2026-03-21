#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__PREDICT_ACTION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__PREDICT_ACTION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sh_bt_base_template/bt_service_node.hpp"
#include "sh_interfaces/srv/detect_objects.hpp"

namespace sh_behavior_tree
{

using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using DetectObjectsSrv = sh_interfaces::srv::DetectObjects;

/**
 * @class sh_behavior_tree::PredictAction
 * @brief BehaviorTree Action Node that calls the detection server and gets detected objects info.
 */
class PredictAction : public sh_bt_base_template::BTServiceNode<
  rclcpp_lifecycle::LifecycleNode, DetectObjectsSrv>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::PredictAction class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit PredictAction(const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::PredictAction class.
   */
  ~PredictAction();

  /**
   * @brief Creates list of BT ports.
   *
   * @return BT::PortsList List of ports with their type and description.
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Method invoked by tick() to send a service request.
   *
   * @param request Service request.
   */
  bool send_request(std::shared_ptr<DetectObjectsSrv::Request>& request);

  /**
   * @brief Receives synchronized messages from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus onTick(
    const std::shared_ptr<DetectObjectsSrv::Response>& response) override;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__PREDICT_ACTION_HPP_