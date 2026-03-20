#ifndef SH_BT_CORE__BT_NODE_LOADER_HPP_
#define SH_BT_CORE__BT_NODE_LOADER_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace sh_bt_core
{

void register_default_nodes(BT::BehaviorTreeFactory & factory);

void register_from_plugin(
  BT::BehaviorTreeFactory & factory,
  const std::vector<std::string> & plugins_ids);

}  // namespace sh_bt_core


#endif  // SH_BT_CORE__BT_NODE_LOADER_HPP_