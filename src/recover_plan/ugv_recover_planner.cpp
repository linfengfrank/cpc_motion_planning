#include "recover_plan/ugv_recover_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/plan_request.h>

UGVRecMotionPlanner::UGVRecMotionPlanner()
{
  m_pso_planner = new PSO::Planner<RECOVER_UGV>(120,50,3);

  // Init swarm
  m_pso_planner->m_swarm.set_dt(0.5f);
  m_pso_planner->m_swarm.set_var(make_float3(2.0f,1.0f,1.0f));
  m_pso_planner->m_file_location = "/home/sp/cpc_ws/ugv_1030/";
  m_pso_planner->initialize();

  m_curr_action_id = 0;

  ros::NodeHandle nh;
  m_carrot_pub = nh.advertise<geometry_msgs::PoseStamped>("/carrot",1);
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

  //std::cout<<"MMMMMMMMMMMMMMM: "<<(int)m_pso_planner->m_eva.m_mode<<std::endl;
  geometry_msgs::PoseStamped carrot_msg;
  carrot_msg.header.frame_id="world";
  carrot_msg.pose.position.x = m_pso_planner->m_eva.m_goal.p.x;
  carrot_msg.pose.position.y = m_pso_planner->m_eva.m_goal.p.y;
  carrot_msg.pose.position.z = 0;
  m_carrot_pub.publish(carrot_msg);

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
  if (path.actions.size() > 0)
  {
    m_path = path;
    m_curr_action_id = 0;
  }
}

UGV::UGVModel::State UGVRecMotionPlanner::calculate_tgt_state(const cpc_motion_planning::path_action &pa)
{
  UGV::UGVModel::State gs; //goal state
  clear_collision_checking_path();
  switch (pa.type)
  {
  case 0: // should not happen
    ROS_ERROR("case 0 action appeared");
    collision_checking_path = pa;
    break;
    //---
  case 1: // turning
  {
    gs.p.x = pa.x[0];
    gs.p.y = pa.y[0];
    gs.theta = find_turning_angle(pa);
    populate_collision_checking_path_turning(gs.p,  m_pso_planner->m_model.get_ini_state().theta, gs.theta);
    break;
  }
    //---
  case 2:
  {
    size_t start_id, end_id;
    gs = find_carrot(pa, start_id, end_id);
    populate_collision_checking_path_moving(pa,start_id,end_id);
    m_pso_planner->m_eva.line_a = make_float2(pa.x[start_id], pa.y[start_id]);
    m_pso_planner->m_eva.line_b = make_float2(pa.x[end_id], pa.y[end_id]);
    break;
  }
    //---
  case 3:
  {
    size_t start_id, end_id;
    gs = find_carrot(pa, start_id, end_id);
    populate_collision_checking_path_moving(pa,start_id,end_id);
    gs.theta += M_PI;
    m_pso_planner->m_eva.line_a = make_float2(pa.x[start_id], pa.y[start_id]);
    m_pso_planner->m_eva.line_b = make_float2(pa.x[end_id], pa.y[end_id]);
  }
  default:
    break;
  }

  //std::cout<<"col check path: "<<collision_checking_path.x.size()<<std::endl;
  return gs;
}

UGV::UGVModel::State UGVRecMotionPlanner::find_carrot(const cpc_motion_planning::path_action &pa, size_t &line_idx_a, size_t &line_idx_b)
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

  size_t carrot_id = min_id;
  for (size_t i = min_id; i < pa.x.size(); i++)
  {
    diff = make_float2(pa.x[i] - s.p.x, pa.y[i]-s.p.y);
    diff_dist = sqrtf(dot(diff,diff));

    if (diff_dist < 2.5f && !is_curvature_too_big(pa, min_id, i))
      carrot_id = i;
    else
      break;
  }

  line_idx_a = min_id;
  line_idx_b = carrot_id;

  // calculate the carrot
  carrot.p.x = pa.x[carrot_id];
  carrot.p.y = pa.y[carrot_id];
  diff = make_float2(pa.x[carrot_id] - s.p.x, pa.y[carrot_id]-s.p.y);
  carrot.theta = atan2f(diff.y, diff.x);

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
    float yaw_diff = s.theta - m_pso_planner->m_eva.m_goal.theta;//pa.theta.back();
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

float UGVRecMotionPlanner::find_turning_angle(const cpc_motion_planning::path_action &pa)
{
  //find the distance of the first and the last point
  float2 diff = make_float2(pa.x.back() - pa.x.front(), pa.y.back()-pa.y.front());
  float diff_dist = sqrtf(dot(diff,diff));
  if (diff_dist > 0.19f)
  {
    float proposed_theta = atan2f(diff.y,diff.x);
    float proposed_theta_rev = proposed_theta + M_PI;

    //find the best match with pa.theta.back();
    float err = fabsf(in_pi(pa.theta.back()-proposed_theta));
    float err_rev = fabsf(in_pi(pa.theta.back()-proposed_theta_rev));

    if (err < err_rev)
      return proposed_theta;
    else
      return proposed_theta_rev;
  }
  else
  {
    return pa.theta.back();
  }
}

bool UGVRecMotionPlanner::is_curvature_too_big(const cpc_motion_planning::path_action &pa, size_t start, size_t end)
{
  float3 start_point = make_float3(pa.x[start], pa.y[start], 0);
  float3 end_point = make_float3(pa.x[end], pa.y[end], 0);
  float3 test_point;
  float max_deviation = 0;
  float deviation;
  for (size_t i = start; i<=end; i++)
  {
    test_point = make_float3(pa.x[i], pa.y[i], 0);
    deviation = pnt2line_dist(start_point, end_point, test_point);

    if (deviation > max_deviation)
      max_deviation = deviation;
  }

  if (max_deviation > 0.15f)
    return true;
  else
    return false;
}
