#include <thor_mang_footstep_planning_plugins/thor_mang_heuristic.h>



namespace thor_mang_footstep_planning
{
using namespace l3_footstep_planning;

ThorMangHeuristic::ThorMangHeuristic()
: HeuristicPlugin("thor_mang_heuristic")
{}

bool ThorMangHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!HeuristicPlugin::loadParams(params))
    return false;

  getParam("step_cost", step_cost_, 0.1);
  getParam("diff_angle_cost", diff_angle_cost_, 0.0);

  getParam("max_step_dist_x", max_step_dist_x_inv_, 0.1);
  max_step_dist_x_inv_ = 1.0/max_step_dist_x_inv_;
  getParam("max_step_dist_y", max_step_dist_y_inv_, 0.1);
  max_step_dist_y_inv_ = 1.0/max_step_dist_y_inv_;

  return true;
}

double ThorMangHeuristic::getHeuristicValue(const Foothold& from, const Foothold& to, const State& /*start*/, const State& /*goal*/) const
{
  if (from == to)
    return 0.0;

  // determine swing dist
  Transform dstep = Foothold::getDelta(from, to);
  double expected_steps_x = std::abs(dstep.x()) * max_step_dist_x_inv_;
  double expected_steps_y = std::abs(dstep.y()) * max_step_dist_y_inv_;

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
    return std::abs(dstep.yaw()) * diff_angle_cost_;
  }

  return 0.0;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thor_mang_footstep_planning::ThorMangHeuristic, l3_footstep_planning::HeuristicPlugin)

