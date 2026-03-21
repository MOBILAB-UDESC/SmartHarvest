#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__CLEAR_PLANNING_SCENE_ACTION_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__CLEAR_PLANNING_SCENE_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_bt_base_template/bt_service_node.hpp"
#include "sh_interfaces/srv/clear_planning_scene.hpp"

namespace sh_behavior_tree
{

using ClearPlanningSceneSrv = sh_interfaces::srv::ClearPlanningScene;

/**
 * @class sh_behavior_tree::ClearPlanningSceneAction
 * @brief BehaviorTree Action Node that clears MoveIt 2 planning scene.
 */
class ClearPlanningSceneAction :
  public sh_bt_base_template::BTServiceNode<rclcpp_lifecycle::LifecycleNode, ClearPlanningSceneSrv>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::ClearPlanningSceneAction class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit ClearPlanningSceneAction(
    const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::ClearPlanningSceneAction class.
   */
  ~ClearPlanningSceneAction();

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
  bool send_request(std::shared_ptr<ClearPlanningSceneSrv::Request>& request);

  /**
   * @brief Receives response from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus onTick(
    const std::shared_ptr<ClearPlanningSceneSrv::Response>& response) override;
};

}  // namespace sh_behavior_tree


#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__CLEAR_PLANNING_SCENE_ACTION_HPP_