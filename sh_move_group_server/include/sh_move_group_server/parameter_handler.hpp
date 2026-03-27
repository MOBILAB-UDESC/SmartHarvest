#ifndef SH_MOVE_GROUP_SERVER__PARAMETER_HANDLER_HPP_
#define SH_MOVE_GROUP_SERVER__PARAMETER_HANDLER_HPP_

#include <string>
#include <vector>

namespace sh_move_group_server
{

struct MoveGroupParameter
{
  std::string move_group;
  std::string planner_id, planning_pipeline;
  double planning_time;
  int planning_attempts;
  std::vector<double> vel_acc_scaling_factors;
  std::map<std::string, std::string> planner_params;
  bool override;
};  // struct MoveGroupParameter

struct Parameter
{
  MoveGroupParameter arm;
  MoveGroupParameter gripper;
};  // struct Parameter

}  // namespace sh_move_group_server


#endif  // SH_MOVE_GROUP_SERVER__PARAMETER_HANDLER_HPP_