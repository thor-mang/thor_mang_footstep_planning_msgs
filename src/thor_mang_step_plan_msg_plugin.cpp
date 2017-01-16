#include <thor_mang_footstep_planning_plugins/thor_mang_step_plan_msg_plugin.h>

#include <vigir_footstep_planning_msgs/step_plan.h>

#include <thormang3_walking_module_msgs/StepPositionData.h>
#include <thormang3_walking_module_msgs/StepTimeData.h>



namespace thor_mang_footstep_planning
{
using namespace vigir_footstep_planning;

static constexpr double FOOT_Z = -0.63;

ThorMangStepPlanMsgPlugin::ThorMangStepPlanMsgPlugin()
  : StepPlanMsgPlugin("thor_mang_step_plan_msg_plugin")
{
}

ThorMangStepPlanMsgPlugin::~ThorMangStepPlanMsgPlugin()
{
}

void initStepData(robotis_framework::StepData& step_data)
{
  step_data.position_data.left_foot_pose.x = 0.0;
  step_data.position_data.left_foot_pose.y = 0.093;
  step_data.position_data.left_foot_pose.z = FOOT_Z;
  step_data.position_data.left_foot_pose.roll = 0.0;
  step_data.position_data.left_foot_pose.pitch = 0.0;
  step_data.position_data.left_foot_pose.yaw = 0.0;

  step_data.position_data.right_foot_pose.x = 0.0;
  step_data.position_data.right_foot_pose.y = -0.093;
  step_data.position_data.right_foot_pose.z = FOOT_Z;
  step_data.position_data.right_foot_pose.roll = 0.0;
  step_data.position_data.right_foot_pose.pitch = 0.0;
  step_data.position_data.right_foot_pose.yaw = 0.0;

  step_data.position_data.body_pose.z = 0.0;
  step_data.position_data.body_pose.roll = 0.0;
  step_data.position_data.body_pose.pitch = 0.0;
  step_data.position_data.body_pose.yaw = 0.0;

  step_data.position_data.waist_roll_angle = 0.0;
  step_data.position_data.waist_pitch_angle = 0.0;
  step_data.position_data.waist_yaw_angle = 0.0;

  step_data.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
  step_data.position_data.foot_z_swap = 0.0;
  step_data.position_data.body_z_swap = 0.0;

  step_data.position_data.shoulder_swing_gain = 0.0;
  step_data.position_data.elbow_swing_gain = 0.0;

  step_data.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
  step_data.time_data.abs_step_time = 3.2;
  step_data.time_data.dsp_ratio = 0.2;

  step_data.time_data.start_time_delay_ratio_x = 0.0;
  step_data.time_data.start_time_delay_ratio_y = 0.0;
  step_data.time_data.start_time_delay_ratio_z = 0.0;
  step_data.time_data.start_time_delay_ratio_roll = 0.0;
  step_data.time_data.start_time_delay_ratio_pitch = 0.0;
  step_data.time_data.start_time_delay_ratio_yaw = 0.0;

  step_data.time_data.finish_time_advance_ratio_x = 0.0;
  step_data.time_data.finish_time_advance_ratio_y = 0.0;
  step_data.time_data.finish_time_advance_ratio_z = 0.0;
  step_data.time_data.finish_time_advance_ratio_roll = 0.0;
  step_data.time_data.finish_time_advance_ratio_pitch = 0.0;
  step_data.time_data.finish_time_advance_ratio_yaw = 0.0;
}

bool operator<<(robotis_framework::StepData& step_data, const msgs::Step& step)
{
  robotis_framework::Pose3D& swing_foot = step.foot.foot_index == msgs::Foot::LEFT ? step_data.position_data.left_foot_pose : step_data.position_data.right_foot_pose;
  robotis_framework::Pose3D& stand_foot = step.foot.foot_index == msgs::Foot::LEFT ? step_data.position_data.right_foot_pose : step_data.position_data.left_foot_pose;

  toThor(step.foot.pose, swing_foot);

  double foot_dz = swing_foot.z - stand_foot.z;

  // no significant change in z
  if (std::abs(foot_dz) < 0.1)
  {
    step_data.position_data.body_pose.z = stand_foot.z - FOOT_Z;
  }
  // step up
  else if (foot_dz > 0.0)
  {
    // if stand foot is the upper one then don't lift immediatly body to full high
    if (stand_foot.z > swing_foot.z - foot_dz)
    {
      step_data.position_data.body_pose.z = 0.25*(stand_foot.z+swing_foot.z) - FOOT_Z;
    }
    else
    {
      step_data.position_data.body_pose.z = stand_foot.z - FOOT_Z;
    }
  }
  // step down
  else
  {
    step_data.position_data.body_pose.z = swing_foot.z - FOOT_Z;
  }

//  step_data.position_data.foot_z_swap = step_data.position_data.dFootHeight + foot_dz;

  step_data.position_data.body_pose.roll = 0.0;
  step_data.position_data.body_pose.pitch = 0.0;
  step_data.position_data.body_pose.yaw = 0.5*(step_data.position_data.left_foot_pose.yaw + step_data.position_data.right_foot_pose.yaw);

  step_data.position_data.waist_roll_angle = 0.0;
  step_data.position_data.waist_pitch_angle = 0.0;
  step_data.position_data.waist_yaw_angle = 0.0;

  step_data.position_data.moving_foot = step.foot.foot_index == msgs::Foot::LEFT ? static_cast<int>(thormang3_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING) : static_cast<int>(thormang3_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING);
  step_data.position_data.foot_z_swap = step.swing_height;
  step_data.position_data.body_z_swap = 0.01;

  step_data.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING;
  step_data.time_data.abs_step_time += step.step_duration;
  step_data.time_data.dsp_ratio = 0.2;

  return true;
}

bool operator<<(std::vector<robotis_framework::StepData>& step_data_list, const msgs::StepPlan& step_plan)
{
  if (step_plan.steps.size() < 1)
  {
    ROS_ERROR("Got insufficient steps!");
    return false;
  }

  msgs::StepPlan _step_plan = step_plan;
  robotis_framework::StepData step_data_curr;

  // generate new plan from scratch
  if (step_data_list.empty())
  {
    // transform plan to be relative to reference start foot pose
    tf::Pose pose;
    tf::poseMsgToTF(step_plan.steps[0].foot.pose, pose);
    tf::Transform transform = pose.inverse();
    StepPlan::transformStepPlan(_step_plan, transform);

    // estimate current position
    initStepData(step_data_curr);
    toThor(_step_plan.start.left.pose, step_data_curr.position_data.left_foot_pose);
    toThor(_step_plan.start.right.pose, step_data_curr.position_data.right_foot_pose);
    //step_data_curr.position_data.body_pose.z = BODY_HEIGHT + std::min(step_data_curr.position_data.left_foot_pose.z, step_data_curr.position_data.right_foot_pose.z);
    //step_data_curr.position_data.body_pose.yaw = 0.5*(step_data_curr.position_data.right_foot_pose.yaw + step_data_curr.position_data.left_foot_pose.yaw);

    // add start walking entry
    step_data_list.push_back(step_data_curr);
  }
  // stitch plan
  else
  {
    robotis_framework::StepData& ref_step_data = step_data_list.back();
    const msgs::Step& step = step_plan.steps[0];

    // determine thor's internal start foot position
    robotis_framework::Pose3D ref_thor_foot;
    if (ref_step_data.position_data.moving_foot == thormang3_walking_module_msgs::StepPositionData::STANDING)
    {
      if (step.foot.foot_index == msgs::Foot::RIGHT)
        ref_thor_foot = ref_step_data.position_data.right_foot_pose;
      else
        ref_thor_foot = ref_step_data.position_data.left_foot_pose;
    }
    else if (ref_step_data.position_data.moving_foot == thormang3_walking_module_msgs::StepPositionData::RIGHT_FOOT_SWING && step.foot.foot_index == msgs::Foot::RIGHT)
      ref_thor_foot = ref_step_data.position_data.right_foot_pose;
    else if (ref_step_data.position_data.moving_foot == thormang3_walking_module_msgs::StepPositionData::LEFT_FOOT_SWING && step.foot.foot_index == msgs::Foot::LEFT)
      ref_thor_foot = ref_step_data.position_data.left_foot_pose;
    else
    {
      ROS_ERROR("Start foot in step plan doesn't match the last moving foot of current plan!");
      return false;
    }

    // ref foot pose of thor's internal walking engine
    tf::Pose ref_thor_foot_pose;
    toRos(ref_thor_foot, ref_thor_foot_pose);

    // ref foot pose of plan
    tf::Pose ref_plan_foot_pose;
    tf::poseMsgToTF(step.foot.pose, ref_plan_foot_pose);
    ref_plan_foot_pose.setRotation(tf::createQuaternionFromYaw(tf::getYaw(ref_plan_foot_pose.getRotation()))); /// HACK to clamp everything to flat ground

    // get transformation 'footstep plan start' -> 'thor start foot'
    tf::Transform transform = ref_thor_foot_pose * ref_plan_foot_pose.inverse();

    // transform plan to be relative to thor's reference foot pose
    StepPlan::transformStepPlan(_step_plan, transform);

    // check if initial state is needed
    if (ref_step_data.time_data.walking_state == thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING)
    {
      ref_step_data.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_STARTING;
      ref_step_data.time_data.abs_step_time += 3.2;
    }
    else // already walking, initial state isn't needed
      step_data_list.clear();

    step_data_curr = ref_step_data;
  }

  std::vector<msgs::Step>::const_iterator itr = _step_plan.steps.begin();
  itr++; // skip inital step of step plan
  for (; itr != _step_plan.steps.end(); itr++)
  {
    // update with next step
    step_data_curr << *itr;
    step_data_list.push_back(step_data_curr);
  }

  // append last step data again as ending
  step_data_curr.position_data.moving_foot = thormang3_walking_module_msgs::StepPositionData::STANDING;
  step_data_curr.time_data.walking_state = thormang3_walking_module_msgs::StepTimeData::IN_WALKING_ENDING;
  step_data_curr.time_data.abs_step_time += 1.0;
  step_data_list.push_back(step_data_curr);

  return true;
}

/*
bool operator==(const Thor::StepData& lhs, const Thor::StepData& rhs)
{
  return (lhs.position_data.moving_foot == rhs.position_data.moving_foot &&
          lhs.position_data.dFootHeight == rhs.position_data.dFootHeight &&
          lhs.position_data.dZ_Swap_Amplitude == rhs.position_data.dZ_Swap_Amplitude &&
          lhs.position_data.dShoulderSwingGain == rhs.position_data.dShoulderSwingGain &&
          lhs.position_data.dElbowSwingGain == rhs.position_data.dElbowSwingGain &&

          std::abs(lhs.position_data.left_foot_pose.x - rhs.position_data.left_foot_pose.x) < 10.0 &&
          std::abs(lhs.position_data.left_foot_pose.y - rhs.position_data.left_foot_pose.y) < 10.0 &&
          std::abs(lhs.position_data.left_foot_pose.z - rhs.position_data.left_foot_pose.z) < 10.0 &&
          std::abs(lhs.position_data.left_foot_pose.roll - rhs.position_data.left_foot_pose.roll) < 0.1 &&
          std::abs(lhs.position_data.left_foot_pose.pitch - rhs.position_data.left_foot_pose.pitch) < 0.1 &&
          std::abs(lhs.position_data.left_foot_pose.yaw - rhs.position_data.left_foot_pose.yaw) < 0.1 &&

          std::abs(lhs.position_data.right_foot_pose.x - rhs.position_data.right_foot_pose.x) < 10.0 &&
          std::abs(lhs.position_data.right_foot_pose.y - rhs.position_data.right_foot_pose.y) < 10.0 &&
          std::abs(lhs.position_data.right_foot_pose.z - rhs.position_data.right_foot_pose.z) < 10.0 &&
          std::abs(lhs.position_data.right_foot_pose.roll - rhs.position_data.right_foot_pose.roll) < 0.1 &&
          std::abs(lhs.position_data.right_foot_pose.pitch - rhs.position_data.right_foot_pose.pitch) < 0.1 &&
          std::abs(lhs.position_data.right_foot_pose.yaw - rhs.position_data.right_foot_pose.yaw) < 0.1 &&

          std::abs(lhs.position_data.body_pose.z - rhs.position_data.body_pose.z) < 10.0 &&
          std::abs(lhs.position_data.body_pose.roll - rhs.position_data.body_pose.roll) < 0.1 &&
          std::abs(lhs.position_data.body_pose.pitch - rhs.position_data.body_pose.pitch) < 0.1 &&
          std::abs(lhs.position_data.body_pose.yaw - rhs.position_data.body_pose.yaw) < 0.1 &&

          lhs.time_data.walking_state == rhs.time_data.walking_state &&
          lhs.time_data.abs_step_time == rhs.time_data.abs_step_time &&
          lhs.time_data.dDSPratio == rhs.time_data.dDSPratio &&

          lhs.time_data.sigmoid_ratio_x == rhs.time_data.sigmoid_ratio_x &&
          lhs.time_data.sigmoid_ratio_y == rhs.time_data.sigmoid_ratio_y &&
          lhs.time_data.sigmoid_ratio_z == rhs.time_data.sigmoid_ratio_z &&
          lhs.time_data.sigmoid_ratio_roll == rhs.time_data.sigmoid_ratio_roll &&
          lhs.time_data.sigmoid_ratio_pitch == rhs.time_data.sigmoid_ratio_pitch &&
          lhs.time_data.sigmoid_ratio_yaw == rhs.time_data.sigmoid_ratio_yaw &&

          lhs.time_data.sigmoid_distortion_x == rhs.time_data.sigmoid_distortion_x &&
          lhs.time_data.sigmoid_distortion_y == rhs.time_data.sigmoid_distortion_y &&
          lhs.time_data.sigmoid_distortion_z == rhs.time_data.sigmoid_distortion_z &&
          lhs.time_data.sigmoid_distortion_roll == rhs.time_data.sigmoid_distortion_roll &&
          lhs.time_data.sigmoid_distortion_pitch == rhs.time_data.sigmoid_distortion_pitch &&
          lhs.time_data.sigmoid_distortion_yaw == rhs.time_data.sigmoid_distortion_yaw);
}

bool operator!=(const Thor::StepData& lhs, const Thor::StepData& rhs)
{
  return !(lhs == rhs);
}

bool operator==(const std::vector<Thor::StepData>& lhs, const std::vector<Thor::StepData>& rhs)
{
  if (lhs.size() != rhs.size())
    return false;

  // ignore first entry, as start config may differ
  for (size_t i = 1; i < lhs.size(); i++)
  {
    if (lhs[i] != rhs[i])
      return false;
  }

  return true;
}*/

void toThor(const tf::Pose& pose_in, robotis_framework::Pose3D& pose_out)
{
  geometry_msgs::Pose msg;
  tf::poseTFToMsg(pose_in, msg);
  toThor(msg, pose_out);
}

void toThor(const geometry_msgs::Pose& pose_in, robotis_framework::Pose3D& pose_out)
{
  pose_out.x = pose_in.position.x;
  pose_out.y = pose_in.position.y;
  pose_out.z = FOOT_Z;//pose_in.position.z;
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose_in.orientation, q);
  tf::Matrix3x3(q).getRPY(pose_out.roll, pose_out.pitch, pose_out.yaw);

  /// TODO: Hack as long state estimation doesn't work
  pose_out.roll = pose_out.pitch = 0.0;
}

void toRos(const robotis_framework::Pose3D& pose_in, tf::Pose& pose_out)
{
  geometry_msgs::Pose msg;
  toRos(pose_in, msg);
  tf::poseMsgToTF(msg, pose_out);
}

void toRos(const robotis_framework::Pose3D& pose_in, geometry_msgs::Pose& pose_out)
{
  pose_out.position.x = pose_in.x;
  pose_out.position.y = pose_in.y;
  pose_out.position.z = pose_in.z;
  pose_out.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_in.roll, pose_in.pitch, pose_in.yaw);
}

std::string toString(const robotis_framework::StepData& step_data)
{
  std::stringstream s;

  s << std::endl;

  s << "[ Step] " << step_data.position_data.moving_foot << " " /*<< step_data.position_data.dFootHeight << " "<< step_data.position_data.dZ_Swap_Amplitude << " "*/
    << step_data.position_data.shoulder_swing_gain << " " << step_data.position_data.elbow_swing_gain << std::endl;

  s << "[ Left] " << step_data.position_data.left_foot_pose.x << " " << step_data.position_data.left_foot_pose.y << " " <<step_data.position_data.left_foot_pose.z << " "
    << step_data.position_data.left_foot_pose.roll << " " <<step_data.position_data.left_foot_pose.pitch << " " << step_data.position_data.left_foot_pose.yaw << std::endl;

  s << "[Right] " << step_data.position_data.right_foot_pose.x << " " << step_data.position_data.right_foot_pose.y << " " <<step_data.position_data.right_foot_pose.z << " "
    << step_data.position_data.right_foot_pose.roll << " " <<step_data.position_data.right_foot_pose.pitch << " " << step_data.position_data.right_foot_pose.yaw << std::endl;

  s << "[ Body] " << step_data.position_data.body_pose.z << " " << step_data.position_data.body_pose.roll << " " << step_data.position_data.body_pose.pitch << " " << step_data.position_data.body_pose.yaw << std::endl;

  s << "[ Time]" << step_data.time_data.walking_state << " " << step_data.time_data.abs_step_time << " " << step_data.time_data.dsp_ratio << std::endl;

  s << "[Start]" << step_data.time_data.start_time_delay_ratio_x << " " << step_data.time_data.start_time_delay_ratio_y << " " << step_data.time_data.start_time_delay_ratio_z << " "
    << step_data.time_data.start_time_delay_ratio_roll << " " << step_data.time_data.start_time_delay_ratio_pitch << " " << step_data.time_data.start_time_delay_ratio_yaw << std::endl;

  s << "[  End]" << step_data.time_data.finish_time_advance_ratio_x << " " << step_data.time_data.finish_time_advance_ratio_y << " " << step_data.time_data.finish_time_advance_ratio_z << " "
    << step_data.time_data.finish_time_advance_ratio_roll << " " << step_data.time_data.finish_time_advance_ratio_pitch << " " << step_data.time_data.finish_time_advance_ratio_yaw << std::endl;

  return s.str();
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(thor_mang_footstep_planning::ThorMangStepPlanMsgPlugin, vigir_footstep_planning::StepPlanMsgPlugin)

