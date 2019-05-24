#include <thor_mang_footstep_planning_plugins/thor_mang_reachability.h>



namespace thor_mang_footstep_planning
{
ThorMangReachability::ThorMangReachability()
  : ReachabilityPlugin("thor_mang_reachability")
{
}

bool ThorMangReachability::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!ReachabilityPlugin::loadParams(params))
    return false;

  ros::NodeHandle nh;
  nh.getParam("foot/separation", foot_seperation_);

  return true;
}

bool ThorMangReachability::isReachable(const PlanningState& state) const
{
  Step::ConstPtr step = state.getStep();

  if (!step || step->empty())
    return true;

  for (const Step::StepDataPair& p : *step)
  {
    StepData::ConstPtr step_data = p.second;

    // restrict to RTR movement
    if (std::abs(step_data->dx) > 0.01 && std::abs(step_data->dy) > 0.01)
      return false;

    if (std::abs(step_data->dx) > 0.05 && std::abs(step_data->dyaw) > 0.01)
      return false;

    if (std::abs(step_data->dy) > 0.01 && std::abs(step_data->dyaw) > 0.01)
      return false;
  }

  return true;
}

bool ThorMangReachability::isReachable(const State& state) const
{
  return true;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thor_mang_footstep_planning::ThorMangReachability, l3_footstep_planning::ReachabilityPlugin)

