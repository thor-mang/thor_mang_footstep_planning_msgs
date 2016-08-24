//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef THOR_MANG_STEP_PLAN_MSG_PLUGIN_H__
#define THOR_MANG_STEP_PLAN_MSG_PLUGIN_H__

#include <string>
#include <sstream>

#include <ros/ros.h>
#include <tf/tf.h>

#include <vigir_footstep_planning_plugins/plugins/step_plan_msg_plugin.h>

#include <robotis_math/step_data_define.h>



namespace thor_mang_footstep_planning
{
using namespace vigir_footstep_planning;

class ThorMangStepPlanMsgPlugin
  : public StepPlanMsgPlugin
{
public:
  // typedefs
  typedef boost::shared_ptr<ThorMangStepPlanMsgPlugin> Ptr;
  typedef boost::shared_ptr<const ThorMangStepPlanMsgPlugin> ConstPtr;

  ThorMangStepPlanMsgPlugin();
  virtual ~ThorMangStepPlanMsgPlugin();
};

void initStepData(robotis_framework::StepData& step_data);

// serialization wrappers
template<typename T>
inline bool operator<<(std::vector<uint8_t>& data, const T& in)
{
  return vigir_generic_params::operator<<(data, in);
}

template<typename T>
inline bool operator<<(T& out, const std::vector<uint8_t>& data)
{
  return vigir_generic_params::operator<<(out, data);
}

// conversions
bool operator<<(robotis_framework::StepData& step_data, const msgs::Step& step);
bool operator<<(std::vector<robotis_framework::StepData>& step_data_list, const msgs::StepPlan& step_plan);

/*
// other operators
bool operator==(const Thor::StepData& lhs, const Thor::StepData& rhs);
bool operator!=(const Thor::StepData& lhs, const Thor::StepData& rhs);
bool operator==(const std::vector<Thor::StepData>& lhs, const std::vector<Thor::StepData>& rhs);
*/
// some math helper
void toThor(const tf::Pose& pose_in, robotis_framework::Pose3D& pose_out);
void toThor(const geometry_msgs::Pose& pose_in, robotis_framework::Pose3D& pose_out);
void toRos(const robotis_framework::Pose3D& pose_in, tf::Pose& pose_out);
void toRos(const robotis_framework::Pose3D& pose_in, geometry_msgs::Pose& pose_out);

std::string toString(const robotis_framework::StepData& step_data);
}

#endif
