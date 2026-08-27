#include "sh_behavior_tree/plugins/action/navigation/get_mean_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

#include "sh_interfaces/msg/perception_scene.hpp"
#include "sh_interfaces/msg/single_object_info.hpp"

namespace sh_behavior_tree
{

GetMeanPose::GetMeanPose(
  const std::string& action_name, const BT::NodeConfig& config):
  BT::SyncActionNode(action_name, config), logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Node created.");

  node_ = config.blackboard->get<rclcpp_lifecycle::LifecycleNode::WeakPtr>("root_node");
  auto node = node_.lock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList GetMeanPose::providedPorts()
{
  return{
    BT::InputPort<sh_interfaces::msg::PerceptionScene>(
      "perception_scene",
      "Complete perception result containing detected objects, header, and processing metadata"),
    BT::InputPort<double>("safety_dist", "Safety distance from the target."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("nearest_object", "Goal pose"),
  };
}

BT::NodeStatus GetMeanPose::tick()
{
  // TODO: Implement an approach service.
  sh_interfaces::msg::PerceptionScene perception_scene;
  double safety_dist;
  getInput("perception_scene", perception_scene);
  getInput("safety_dist", safety_dist);

  double x_m = 0.0;
  double y_m = 0.0;
  for (const auto& object: perception_scene.objects) {
    x_m += object.pose.position.x;
    y_m += object.pose.position.y;
  }

  x_m /= perception_scene.objects.size();
  y_m /= perception_scene.objects.size();
  double dist = std::sqrt(x_m*x_m + y_m*y_m);

  if (dist == 0) {
    RCLCPP_ERROR(logger_, "Avoiding division by 0.");
    return BT::NodeStatus::FAILURE;
  }

  // double goal_x = (dist - safety_dist) * x_m / dist;
  // double goal_y = (dist - safety_dist) * y_m / dist;

  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header = perception_scene.header;
  // {
  //   pose_in.header.stamp = node_.lock()->now();
  // }
  double goal_yaw = std::atan2(y_m, x_m);
  pose_in.pose.position.x = x_m - safety_dist * std::cos(goal_yaw);
  pose_in.pose.position.y = y_m - safety_dist * std::sin(goal_yaw);
  // pose_in.pose.position.x = goal_x;
  // pose_in.pose.position.y = goal_y;
  pose_in.pose.position.z = 0.0;


  tf2::Quaternion q;
  q.setRPY(0, 0, goal_yaw);
  pose_in.pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::PoseStamped pose_out;
  try {
    pose_out = tf_buffer_->transform(
      pose_in, "map", tf2::durationFromSec(1.0));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(logger_,
      "Could not transform object pose to 'map': %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("nearest_object", pose_out);

  RCLCPP_INFO(logger_, "X: %.3f", pose_out.pose.position.x);
  RCLCPP_INFO(logger_, "Y: %.3f", pose_out.pose.position.y);
  RCLCPP_INFO(logger_, "Yaw: %.3f", tf2::getYaw(pose_out.pose.orientation));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::GetMeanPose>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::GetMeanPose>(
    "GetMeanPose", builder);
}