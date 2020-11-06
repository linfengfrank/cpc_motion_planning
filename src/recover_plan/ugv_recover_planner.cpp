#include "recover_plan/ugv_recover_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/plan_request.h>

UGVRecMotionPlanner::UGVRecMotionPlanner()
{
  m_pso_planner = new PSO::Planner<RECOVER_UGV>(120,50,3);
  m_pso_planner->initialize();

  m_curr_action_id = 0;
}

UGVRecMotionPlanner::~UGVRecMotionPlanner()
{
  delete m_pso_planner;
}

bool UGVRecMotionPlanner::calculate_trajectory(const UGV::UGVModel::State &s, EDTMap *edt_map, std::vector<UGV::UGVModel::State> &traj)
{
  m_pso_planner->m_model.set_ini_state(s);
  m_pso_planner->m_eva.m_mode = m_path.actions[m_curr_action_id].type;
  m_pso_planner->m_eva.setTarget(calculate_tgt_state(m_path.actions[m_curr_action_id]));
  std::cout<<m_pso_planner->m_eva.m_goal.p.x<<" "<<m_pso_planner->m_eva.m_goal.p.y<<" "<<m_pso_planner->m_eva.m_goal.theta<<std::endl;
  m_pso_planner->plan(*edt_map);

  // generate the trajectory
  traj = m_pso_planner->generate_trajectory();

  bool finished = false;

  if(check_action_finish(m_path.actions[m_curr_action_id]))
  {
    if(m_curr_action_id + 1 < m_path.actions.size())
    {
      m_curr_action_id ++;
    }
    else
    {
      finished = true;
    }
  }

  return finished;
}

void UGVRecMotionPlanner::set_path_cell(const cpc_motion_planning::path &path)
{
  m_path = path;
  m_curr_action_id = 0;
}

UGV::UGVModel::State UGVRecMotionPlanner::calculate_tgt_state(const cpc_motion_planning::path_action &pa)
{
  UGV::UGVModel::State gs; //goal state
  switch (pa.type)
  {
  case 0: // should not happen
    ROS_ERROR("case 0 action appeared");
    break;
    //---
  case 1: // turning
    gs.p.x = pa.x[0];
    gs.p.y = pa.y[0];
    gs.theta = pa.theta.back();

//    //find the distance of the first and the last point
//    float2 diff = make_float2(pa.x.back() - pa.x.front(), pa.y.back()-pa.y.front());
//    float diff_dist = sqrtf(dot(diff,diff));
//    if (diff_dist > 0.2f)
//    {
//      float proposed_theta = atan2f(diff.y,diff.x);
//    }
//    else
//    {

//    }

    break;
    //---
  case 2:
    gs = find_carrot(pa);
    break;
    //---
  case 3:
    gs = find_carrot(pa);
    gs.theta += M_PI;
  default:
    break;
  }
  return gs;
}

UGV::UGVModel::State UGVRecMotionPlanner::find_carrot(const cpc_motion_planning::path_action &pa)
{
  UGV::UGVModel::State s = m_pso_planner->m_model.get_ini_state();
  UGV::UGVModel::State carrot;

  float2 diff;
  float diff_dist;
  float min_dist = 1e6;
  size_t min_id = 0;
  //first find the nearest point on the path
  for (size_t i = 0; i < pa.x.size(); i++)
  {
    diff = make_float2(pa.x[i] - s.p.x, pa.y[i]-s.p.y);
    diff_dist = sqrtf(dot(diff,diff));
    if (diff_dist < min_dist)
    {
      min_dist = diff_dist;
      min_id = i;
    }
  }

  // find the carrot
  // first initialize the carrot with the min_id point
  carrot.p.x = pa.x[min_id];
  carrot.p.y = pa.y[min_id];

  // calcuate the angle
  diff = make_float2(pa.x[min_id] - s.p.x, pa.y[min_id]-s.p.y);
  carrot.theta = atan2f(diff.y, diff.x);

  for (size_t i = min_id; i < pa.x.size(); i++)
  {
    diff = make_float2(pa.x[i] - s.p.x, pa.y[i]-s.p.y);
    diff_dist = sqrtf(dot(diff,diff));

    if (diff_dist < 0.5f)
    {
      carrot.p.x = pa.x[i];
      carrot.p.y = pa.y[i];

      // calcuate the angle
      diff = make_float2(pa.x[i] - s.p.x, pa.y[i]-s.p.y);
      carrot.theta = atan2f(diff.y, diff.x);
    }
  }

  return carrot;
}

bool UGVRecMotionPlanner::check_action_finish(const cpc_motion_planning::path_action &pa)
{
  UGV::UGVModel::State s = m_pso_planner->m_model.get_ini_state();
  switch (pa.type)
  {
  case 0: // should not happen
    ROS_ERROR("case 0 action appeared");
    return  false;
    //---
  case 1: // turning
  {
    float yaw_diff = s.theta - pa.theta.back();
    yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
    if(fabsf(yaw_diff) < 0.2f && fabsf(s.w)<0.2f)
      return true;
    else
      return false;
  }
    //---
  case 2:
  {
    float2 p_diff = make_float2(s.p.x - pa.x.back(), s.p.y - pa.y.back());
    if (sqrtf(dot(p_diff,p_diff))<0.2f && fabsf(s.v) < 0.2f)
      return true;
    else
      return false;
  }
    //---
  case 3:
  {
    float2 p_diff = make_float2(s.p.x - pa.x.back(), s.p.y - pa.y.back());
    if (sqrtf(dot(p_diff,p_diff))<0.2f && fabsf(s.v) < 0.2f)
      return true;
    else
      return false;
  }

  default:
    break;
  }
}
