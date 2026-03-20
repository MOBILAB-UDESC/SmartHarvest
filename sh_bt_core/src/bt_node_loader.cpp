#include "sh_bt_core/bt_node_loader.hpp"

#include "sh_behavior_tree/plugins_list.hpp"

namespace sh_bt_core
{

void register_default_nodes(BT::BehaviorTreeFactory & factory)
{
  BT::SharedLibrary loader;

  std::stringstream ss(sh_behavior_tree::BT_BUILTIN_PLUGINS);
  std::string default_plugin;

  while (std::getline(ss, default_plugin, ';')) {
    factory.registerFromPlugin(loader.getOSName(default_plugin));
  }
}

void register_from_plugin(
  BT::BehaviorTreeFactory & factory,
  const std::vector<std::string> & plugin_ids)
{
  BT::SharedLibrary loader;
  std::string plugin_so;

  for (const auto & plugin_id: plugin_ids) {
    plugin_so = loader.getOSName(plugin_id);
    factory.registerFromPlugin(plugin_so);
  }
}

}  // namespace sh_bt_core
