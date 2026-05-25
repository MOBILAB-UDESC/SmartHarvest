#include "sh_behavior_tree/plugins/action/navigation/rotate_robot.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sh_behavior_tree
{

RotateRobot::RotateRobot(const std::string & action_name, const BT::NodeConfig & node_config) :
  sh_base_template::BTSubscriptionNode<
    rclcpp_lifecycle::LifecycleNode, nav_msgs::msg::Odometry>(action_name, node_config)
{
  if (!getInput("max_w", max_w_)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'max_w'.");
    throw std::invalid_argument("Missing required input port 'max_w'.");
  }
  if (!getInput("angle_rad", angle_rad_)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'angle_rad'.");
    throw std::invalid_argument("Missing required input port 'angle_rad'.");
  }
  if (!getInput("angle_tolerance", angle_tolerance_)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'angle_tolerance'.");
    throw std::invalid_argument("Missing required input port 'angle_tolerance'.");
  }
  if (!getInput("kp", kp_)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'kp'.");
    throw std::invalid_argument("Missing required input port 'kp'.");
  }
  if (!getInput("kp", ki_)) {
    RCLCPP_ERROR(logger_, "Missing required input port 'ki'.");
    throw std::invalid_argument("Missing required input port 'ki'.");
  }

  integral_error_ = 0.0;

  twist_pub_ = node_.lock()->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/jackal_drive_base_controller/cmd_vel", 10);
}

BT::PortsList RotateRobot::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("max_w", "Max absolute angular velocity (rad/s)."),
    BT::InputPort<double>("angle_rad",
      "Signed relative yaw target added to current yaw at first tick (rad)."),
    BT::InputPort<double>("angle_tolerance", "Goal angle tolerance (rad)."),
    BT::InputPort<double>("kp", "Proportional gain for yaw controller."),
    BT::InputPort<double>("ki", "Integral gain for yaw controller."),
  });
}

BT::NodeStatus RotateRobot::on_timeout()
{
  timeout_count_++;
  RCLCPP_WARN(logger_, "Timeout");
  if (timeout_count_ < 3) {
    return BT::NodeStatus::RUNNING;
  }
  timeout_count_ = 0;
  integral_error_ = 0.0;
  RCLCPP_ERROR(logger_, "Timeout");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RotateRobot::on_tick(const std::shared_ptr<nav_msgs::msg::Odometry>& last_msg)
{
  timeout_count_ = 0;
  double current_yaw = tf2::getYaw(last_msg->pose.pose.orientation);

  if (!goal_initialized_) {
    integral_error_ = 0.0;
    goal_yaw_ = current_yaw + angle_rad_;
    goal_initialized_ = true;
  }

  double yaw_delta = goal_yaw_ - current_yaw;
  while (yaw_delta > M_PI) yaw_delta -= 2 * M_PI;
  while (yaw_delta < -M_PI) yaw_delta += 2 * M_PI;
  RCLCPP_DEBUG(logger_, "Yaw delta: %.3f", yaw_delta);

  integral_error_ += 0.033 * yaw_delta;

  if (std::abs(yaw_delta) <= angle_tolerance_) {
    goal_initialized_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  double angular_velocity = yaw_delta*kp_ + integral_error_*ki_;
  if (angular_velocity > max_w_) {
    angular_velocity = max_w_;
  } else if (angular_velocity < -max_w_) {
    angular_velocity = -max_w_;
  }
  RCLCPP_DEBUG(logger_, "Ang. vel: %.3f", angular_velocity);

  geometry_msgs::msg::TwistStamped msg;
  msg.header = last_msg->header;
  msg.header.stamp = node_.lock()->now();
  msg.twist.angular.z = angular_velocity;

  twist_pub_->publish(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  return BT::NodeStatus::RUNNING;
}

}  // namespace sh_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<sh_behavior_tree::RotateRobot>(name, config);
    };

  factory.registerBuilder<sh_behavior_tree::RotateRobot>("RotateRobot", builder);
}