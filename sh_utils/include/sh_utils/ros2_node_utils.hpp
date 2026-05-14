#ifndef SH_UTILS__ROS2_NODE_UTILS_HPP_
#define SH_UTILS__ROS2_NODE_UTILS_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace sh_utils
{

namespace ros2_node_utils
{

template <class ParamType>
inline void declare_parameter_if_not_declared(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const std::string& param_name,
  const ParamType& default_value)
{
  if(!node->has_parameter(param_name)) {
    node->declare_parameter<ParamType>(param_name, default_value);
  }
}

template <class ParamType>
inline void declare_and_get_parameter(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const std::string& param_name,
  ParamType& value,
  const ParamType& default_value)
{
  declare_parameter_if_not_declared<ParamType>(node, param_name, default_value);

  node->get_parameter<ParamType>(param_name, value);
}

}  // namespace ros2_node_utils

}  // namespace sh_utils

#endif  // SH_UTILS__ROS2_NODE_UTILS_HPP_