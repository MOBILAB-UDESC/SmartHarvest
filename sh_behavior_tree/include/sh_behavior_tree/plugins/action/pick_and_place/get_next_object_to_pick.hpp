#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__GET_NEXT_OBJECT_TO_PICK_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__GET_CURRENT_OBJECT_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_base_template/bt_service_node.hpp"
#include "sh_interfaces/srv/select_next_target.hpp"

namespace sh_behavior_tree
{

using SelectNextTargetSrv = sh_interfaces::srv::SelectNextTarget;

/**
 * @class sh_behavior_tree::GetNextObjectToPick
 * @brief BehaviorTree Action Node that gets the name of the next detected object to pick.
 */
class GetNextObjectToPick :
  public sh_base_template::BTServiceNode<rclcpp_lifecycle::LifecycleNode, SelectNextTargetSrv>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::GetNextObjectToPick class.
   *
   * @param action_name Name of the action node in the BehaviorTree
   * @param config Configuration of the BehaviorTree Node
   */
  explicit GetNextObjectToPick(const std::string& action_name, const BT::NodeConfig& config);

  /**
   * @brief A destructor for sh_behavior_tree::GetNextObjectToPick class.
   */
  ~GetNextObjectToPick() = default;

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
   * @param request Service request to send.
   */
  bool send_request(std::shared_ptr<SelectNextTargetSrv::Request>& request);

  /**
   * @brief Receives response from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus onTick(
    const std::shared_ptr<SelectNextTargetSrv::Response>& response) override;

  // ROS 2 node
  rclcpp::Logger logger_;
};

}  // namespace sh_behavior_tree

#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__GET_CURRENT_OBJECT_HPP_