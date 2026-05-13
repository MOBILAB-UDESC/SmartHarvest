#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__PREDICT_SERVICE_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__PREDICT_SERVICE_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sh_base_template/bt_service_node.hpp"
#include "sh_interfaces/srv/run_perception.hpp"

namespace sh_behavior_tree
{

using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using RunPerceptionSrv = sh_interfaces::srv::RunPerception;

/**
 * @class sh_behavior_tree::PredictService
 * @brief BehaviorTree Action Node that calls the detection server and gets detected objects info.
 */
class PredictService : public sh_base_template::BTServiceNode<
  rclcpp_lifecycle::LifecycleNode, RunPerceptionSrv>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::PredictService class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit PredictService(const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::PredictService class.
   */
  ~PredictService();

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
  bool send_request(std::shared_ptr<RunPerceptionSrv::Request>& request);

  /**
   * @brief Receives service response from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus onTick(
    const std::shared_ptr<RunPerceptionSrv::Response>& response) override;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__DETECTION__PREDICT_SERVICE_HPP_