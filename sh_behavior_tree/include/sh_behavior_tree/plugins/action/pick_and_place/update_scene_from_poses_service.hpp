#ifndef SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__UPDATE_SCENE_FROM_POSES_SERVICE_HPP_
#define SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__UPDATE_SCENE_FROM_POSES_SERVICE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sh_bt_base_template/bt_service_node.hpp"
#include "sh_interfaces/srv/update_planning_scene_from_poses.hpp"

namespace sh_behavior_tree
{

using UpdatePlanningSceneFromPosesSrv = sh_interfaces::srv::UpdatePlanningSceneFromPoses;

/**
 * @class sh_behavior_tree::UpdateSceneFromPosesService
 * @brief BehaviorTree Action Node that adds detected objects into the MoveIt 2 planning scene.
 */
class UpdateSceneFromPosesService :
  public sh_bt_base_template::BTServiceNode<
    rclcpp_lifecycle::LifecycleNode, UpdatePlanningSceneFromPosesSrv>
{
public:
  /**
   * @brief A constructor for sh_behavior_tree::UpdateSceneFromPosesService class.
   *
   * @param action_name Name of the action node in the BehaviorTree.
   * @param node_config Configuration of the BehaviorTree Node.
   */
  explicit UpdateSceneFromPosesService(
    const std::string & action_name, const BT::NodeConfig & node_config);

  /**
   * @brief A destructor for sh_behavior_tree::UpdateSceneFromPosesService class.
   */
  ~UpdateSceneFromPosesService();

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
  bool send_request(std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Request>& request);

  /**
   * @brief Receives response from tick.
   *
   * @return BT::NodeStatus SUCCESS or FAILURE.
   */
  BT::NodeStatus onTick(
    const std::shared_ptr<UpdatePlanningSceneFromPosesSrv::Response>& response) override;
};

}  // namespace sh_behavior_tree


#endif  // SH_BEHAVIOR_TREE__PLUGINS__ACTION__PICK_AND_PLACE__UPDATE_SCENE_FROM_POSES_SERVICE_HPP_