#include <thor_mang_footstep_planning_plugins/thor_mang_heuristic.h>

#include <pluginlib/class_list_macros.h>



namespace thor_mang_footstep_planning
{
using namespace vigir_footstep_planning;

ThorMangHeuristic::ThorMangHeuristic()
: HeuristicPlugin("thor_mang_heuristic")
{}

bool ThorMangHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  params.getParam("thor_mang_heuristic/step_cost", step_cost_, 0.1);
  params.getParam("thor_mang_heuristic/diff_angle_cost", diff_angle_cost_, 0.1);
  params.getParam("max_step_dist/x", max_step_dist_x_inv_);
  max_step_dist_x_inv_ = 1.0/max_step_dist_x_inv_;
  params.getParam("max_step_dist/y", max_step_dist_y_inv_);
  max_step_dist_y_inv_ = 1.0/max_step_dist_y_inv_;

  return true;
}

double ThorMangHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  // determine swing dist
  tf::Transform dstep;
  getDeltaStep(from.getPose(), to.getPose(), dstep);
  double expected_steps_x = std::abs(dstep.getOrigin().x()) * max_step_dist_x_inv_;
  double expected_steps_y = std::abs(dstep.getOrigin().y()) * max_step_dist_y_inv_;

  if (expected_steps_x > 0.1 && expected_steps_y > 0.1) // rotation suggested
    return expected_steps_y * step_cost_;
    //return expected_steps_y/expected_steps_x * step_cost * gain;
  else if (expected_steps_x > 0.1) // transversal
    return expected_steps_x * step_cost_;
  else if (expected_steps_y > 0.1) // sidewards
    return expected_steps_y * step_cost_;
  else // rotate towards goal orientation
  {
    //ROS_INFO("%f", tf::getYaw(dstep.getRotation()));
    return std::abs(tf::getYaw(dstep.getRotation())) * diff_angle_cost_;
  }

  return 0.0;
}
}

PLUGINLIB_EXPORT_CLASS(thor_mang_footstep_planning::ThorMangHeuristic, vigir_footstep_planning::HeuristicPlugin)
