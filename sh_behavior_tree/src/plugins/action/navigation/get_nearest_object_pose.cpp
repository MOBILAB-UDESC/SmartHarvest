#include "sh_behavior_tree/plugins/action/navigation/get_nearest_object_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

#include "sh_interfaces/msg/detected_objects.hpp"
#include "sh_interfaces/msg/single_object_info.hpp"

namespace sh_behavior_tree
{

GetNearestObjectPose::GetNearestObjectPose(
  const std::string& action_name, const BT::NodeConfig& config):
  BT::SyncActionNode(action_name, config), logger_(rclcpp::get_logger(action_name))
{
  RCLCPP_INFO(logger_, "Node created.");

  node_ = config.blackboard->get<rclcpp_lifecycle::LifecycleNode::WeakPtr>("root_node");
  auto node = node_.lock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList GetNearestObjectPose::providedPorts()
{
  return{
    BT::InputPort<sh_interfaces::msg::DetectedObjects>("objects", "List of detected objects"),
    BT::InputPort<double>("safety_dist", "Safety distance from the target."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("nearest_object", "Goal pose"),
  };
}

BT::NodeStatus GetNearestObjectPose::tick()
{
  sh_interfaces::msg::DetectedObjects objects;
  double safety_dist;
  getInput("objects", objects);
  getInput("safety_dist", safety_dist);

  double dist = 10000.0;
  sh_interfaces::msg::SingleObjectInfo nearest_object;

  for (const auto& object: objects.objects) {
    if (object.distance < dist) {
      dist = object.distance;
      nearest_object = object;
    }
  }

  if (dist == 0) {
    RCLCPP_ERROR(logger_, "Avoiding division by 0.");
    return BT::NodeStatus::FAILURE;
  }

  double goal_x = (dist - safety_dist) * nearest_object.x / dist;
  double goal_y = (dist - safety_dist) * nearest_object.y / dist;

  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header = objects.header;
  {
    pose_in.header.stamp = node_.lock()->now();
  }
  pose_in.pose.position.x = goal_x;
  pose_in.pose.position.y = goal_y;
  pose_in.pose.position.z = 0.0;

  double goal_yaw = std::atan2(nearest_object.y, nearest_object.x);

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
      return std::make_unique<sh_behavior_tree::GetNearestObjectPose>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::GetNearestObjectPose>(
    "GetNearestObjectPose", builder);
}