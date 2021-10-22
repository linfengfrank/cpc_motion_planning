#include "loc_plan/integrated/local_planner_pipeline.h"
#include "loc_plan/integrated/states/start_idle_state.h"
#include "loc_plan/integrated/states/normal_teb_state.h"
#include "loc_plan/integrated/states/normal_pso_state.h"
#include "loc_plan/integrated/states/stuck_state.h"
#include "loc_plan/integrated/states/brake_state.h"
#include "tf/tf.h"


LocalPlannerPipeline::LocalPlannerPipeline()
{
  NormalTebState::getInstance().attach_to_pipe(this);
  StartIdleState::getInstance().attach_to_pipe(this);
  NormalPsoState::getInstance().attach_to_pipe(this);
  StuckState::getInstance().attach_to_pipe(this);
  BrakeState::getInstance().attach_to_pipe(this);

  m_state = &StartIdleState::getInstance();
  start_timer();
}

void LocalPlannerPipeline::prepare_cycle()
{
  m_bb.m_init_state = prepare_init_state();
}

void LocalPlannerPipeline::finish_cycle()
{
  // Publish the result to the controller
  // At idle state, we do not publish anything, because the SLAM might not be working yet.
  if (m_state != &StartIdleState::getInstance())
  {
    m_bb.m_stamped_ref_traj = make_stamped_reference(m_ref_start_idx, m_bb.m_ref_traj);
    update_norminal_states(m_bb.m_stamped_ref_traj);
    m_bb.publish_reference(m_bb.m_stamped_ref_traj);
#ifdef PRED_STATE
    update_reference_log(m_bb.m_ref_msg, m_cycle_start_time);
#endif
    increase_ref_gen_count();
#ifdef SHOW_PC
    m_bb.plot_ref_trajectory(m_bb.m_ref_traj);
#endif
  }
}

//---------------------------------------------------------
// Helper functions:
//---------------------------------------------------------
float LocalPlannerPipeline::get_heading(const nav_msgs::Odometry &odom)
{
  double phi,theta,psi;
  tf::Quaternion q(odom.pose.pose.orientation.x,
                   odom.pose.pose.orientation.y,
                   odom.pose.pose.orientation.z,
                   odom.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);
  return psi;
}

UGV::UGVModel::State LocalPlannerPipeline::prepare_init_state()
{
  bool is_heading_ref;
  float psi;

  if(m_bb.m_use_adrc)
  {
    psi = get_heading(m_bb.m_slam_odo);
    is_heading_ref = false;
  }
  else
  {
    psi = select_mes_ref_heading(is_heading_ref,get_heading(m_bb.m_slam_odo),
                                 m_ref_theta, 0.25f);
  }

  UGV::UGVModel::State s = predict_state(m_bb.m_slam_odo,psi,m_ref_start_idx,is_heading_ref);

  s.v = select_mes_ref(m_bb.m_raw_odo.twist.twist.linear.x, m_ref_v);
  s.w = select_mes_ref(m_bb.m_raw_odo.twist.twist.angular.z, m_ref_w);

  return s;
}

// Select wether to use measurement or reference to be assigned to the current state
float LocalPlannerPipeline::select_mes_ref(float mes, float ref, float th, int ctt_th)
{
  static int ctt = 0;
  float output;
  float err = ref - mes;

  if (fabsf(err) > th)
    ctt++;
  else
    ctt = 0;

  if (ctt >= ctt_th)
  {
    ctt = 0;
    output = mes + sgn<float>(err)*0.5f;
  }
  else
  {
    output = ref;
  }
  return output;
}

// Select wether to use measurement or reference to be assigned to the current state (yaw)
float LocalPlannerPipeline::select_mes_ref_heading(bool &is_heading_ref, float mes, float ref, float th, int ctt_th)
{
  static int ctt = 0;
  float output;
  float err = in_pi(ref - mes);

  if (fabsf(err) > th)
    ctt++;
  else
    ctt = 0;

  if (ctt >= ctt_th)
  {
    ctt = 0;
    output = mes + sgn<float>(err)*0.5f;
    is_heading_ref = false;
  }
  else
  {
    output = ref;
    is_heading_ref = true;
  }
  return in_pi(output);
}

UGV::UGVModel::State LocalPlannerPipeline::predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx, bool is_heading_ref)
{
  UGV::UGVModel::State s;
  s.p.x = odom.pose.pose.position.x;
  s.p.y = odom.pose.pose.position.y;
  s.s = 0;
  s.theta = psi;
#ifdef PRED_STATE
  // Find the most related cmd in time
  while (m_cmd_q.size()>0)
  {
    if (m_cmd_q.front().t.toSec() <= odom.header.stamp.toSec())
      m_cmd_q.pop_front();
    else
      break;
  }

  // Use the cmd to perform forward simulation to predicte the state until the ref_start_idx
  for (const CmdLog &tmp : m_cmd_q)
  {
    if (tmp.id < ref_start_idx)
    {
      if (!is_heading_ref)
      {
        s.p.x = s.p.x + tmp.v*cos(s.theta)*PSO::PSO_CTRL_DT;
        s.p.y = s.p.y + tmp.v*sin(s.theta)*PSO::PSO_CTRL_DT;
        s.theta = s.theta + tmp.w*PSO::PSO_CTRL_DT;
      }
      else
      {
        s.p.x = s.p.x + tmp.v*cos(tmp.theta)*PSO::PSO_CTRL_DT;
        s.p.y = s.p.y + tmp.v*sin(tmp.theta)*PSO::PSO_CTRL_DT;
      }
    }
    else
    {
      break;
    }
  }
#endif
  return s;
}

// Generate a full stop trajectory, stop at the current state (curr_s)
void LocalPlannerPipeline::full_stop_trajectory(std::vector<UGV::UGVModel::State> &traj, UGV::UGVModel::State curr_s)
{
  traj.clear();
  float dt = PSO::PSO_CTRL_DT;
  curr_s.v = 0;
  curr_s.w = 0;
  for (float t=0.0f; t<4; t+=dt)
  {
    traj.push_back(curr_s);
  }
}

bool LocalPlannerPipeline::is_tracking_safe(const nav_msgs::Odometry &odom, const std::vector<UGV::UGVModel::State> &ref)
{
  // If not in ADRC mode (tracking mode), the simulated trajectory might collide with obstacle
  if(!m_bb.m_use_adrc && tracking_min_edt(odom, ref, 2, m_bb.m_turning_efficiency) < PSO::MIN_DIST)
    return false;
  else
    return true;
}

float LocalPlannerPipeline::tracking_min_edt(const nav_msgs::Odometry &odom, const std::vector<UGV::UGVModel::State> &ref,
                                                      float yaw_ctrl_gain, float w_scale, float exam_time)
{
  // Simulate the trajectory tracking process
  std::vector<UGV::UGVModel::State> sim_traj = simulate_tracking(ref,odom,yaw_ctrl_gain,w_scale,exam_time);

  // Find the smallest distance to obstacle
  float min_dist = traj_min_edt(sim_traj);

#ifdef SHOW_PC
  m_bb.plot_sim_trajectory(sim_traj);
#endif

  return min_dist;
}

std::vector<UGV::UGVModel::State> LocalPlannerPipeline::simulate_tracking(const std::vector<UGV::UGVModel::State> &ref, const nav_msgs::Odometry &odom,
                                                              float yaw_ctrl_gain, float w_scale, float exam_time)
{
  std::vector<UGV::UGVModel::State> output;
  //Get the true state from odom information
  UGV::UGVModel::State s;
  s.p.x = odom.pose.pose.position.x;
  s.p.y = odom.pose.pose.position.y;
  s.s = 0;
  s.theta = get_heading(odom);

  //Simulating the tracking of the trajectory
  float dt = PSO::PSO_CTRL_DT;
  for (int i=0; i*dt<=exam_time && i < ref.size(); i++)
  {
    s.v = ref[i].v;
    // For the yaw, we have a controller, and a simulated slip (w_scale)
    // The controller shall be exactly the same as the controller in the cpc_ref_publisher pacakge (main.cpp)
    s.w = ref[i].w/w_scale + yaw_ctrl_gain * in_pi(ref[i].theta - s.theta);
    s.w= s.w> 0.6? 0.6:s.w;
    s.w= s.w< -0.6? -0.6:s.w;
    s.w *= w_scale;

    //s and theta
    s.s = s.s + s.v*dt;
    s.theta = s.theta + s.w*dt;

    // x and y
    s.p.x = s.p.x + (s.v*dt)*cos(s.theta + s.w*dt);
    s.p.y = s.p.y + (s.v*dt)*sin(s.theta + s.w*dt);

    output.push_back(s);
  }
  return output;
}

float LocalPlannerPipeline::traj_min_edt(const std::vector<UGV::UGVModel::State> &traj)
{
  // Find the smallest distance to obstacle
  float min_dist = 1000;
  for (const UGV::UGVModel::State &s : traj)
  {
    //Get dist to obstacle for current state s
    float dist = get_dist_from_host_edt(s);
    if (dist < min_dist)
      min_dist = dist;
  }
  return min_dist;
}

// Given a vehicle state, check its minimum distace to obstacles
float LocalPlannerPipeline::get_dist_from_host_edt(const UGV::UGVModel::State &s) const
{
  float2 c_f,c_r;
  calculate_bounding_centres(s, c_r, c_f);
  return min(m_bb.m_edt_map->getEDT(c_r), m_bb.m_edt_map->getEDT(c_f));
}

// Calculate the center position of the bounding circles of the 2 circle model of the vehicle
void LocalPlannerPipeline::calculate_bounding_centres(const UGV::UGVModel::State &s, float2 &c_r, float2 &c_f) const
{
  float2 uni_dir = make_float2(cosf(s.theta),sinf(s.theta));
  c_f = s.p + m_bb.m_footprint_offset*uni_dir;
  c_r = s.p - m_bb.m_footprint_offset*uni_dir;
}

bool LocalPlannerPipeline::is_stuck(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state)
{
  // update stuck probability
  if (is_stuck_instant(traj,tgt_state))
      m_stuck_pbty +=0.15f;
  else
      m_stuck_pbty *=0.8f;

  if (m_stuck_pbty > 1)
  {
    // Remeber to set all forms of stuck probability
    m_lowpass_stuck_pbty = 0;
    m_stuck_pbty = 0;
    return true;
  }
  else
  {
    return false;
  }
}

bool LocalPlannerPipeline::is_stuck_instant(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state)
{
  bool far_from_tgt = false;
  bool no_turning = false;
  bool no_moving_intention = false;

  // check is it far from target
  if (!is_pos_reached(traj[0],tgt_state))
      far_from_tgt = true;

  // check whether the vehicle is about to move
  float max_dist = 0;
  float dist;
  UGV::UGVModel::State ini_s = traj[0];
  float2 p_shift = make_float2(0,0);
  float max_turn = 0;
  float turn;
  for (UGV::UGVModel::State s : traj)
  {
      p_shift = ini_s.p - s.p;
      dist = sqrtf(dot(p_shift,p_shift));
      if (dist > max_dist)
          max_dist = dist;

      turn = fabsf(s.theta - ini_s.theta);
      if (turn > max_turn)
          max_turn = turn;
  }
  if (max_dist < 0.4f)
      no_moving_intention = true;

  if (max_turn < 0.25f)
    no_turning = true;

  return far_from_tgt && no_turning && no_moving_intention;
}

bool LocalPlannerPipeline::is_stuck_lowpass(const UGV::UGVModel::State& s, const UGV::UGVModel::State &tgt_state)
{
  static float lowpass_v = 0.0f;
  static float lowpass_w = 0.0f;
  // Use a low pass filter on the current velocity
  lowpass_v = 0.8f*lowpass_v + 0.2f*s.v;
  lowpass_w = 0.8f*lowpass_w + 0.2f*s.w;

  // If the current linear and rotational speed are both very small, increase the stuck probability
  if (fabsf(lowpass_v) < 0.08f && fabsf(lowpass_w) < 0.08f && !is_pos_reached(s,tgt_state))
    m_lowpass_stuck_pbty +=0.1f;
  else
    m_lowpass_stuck_pbty *=0.8f;

  //std::cout<<"****"<<m_lowpass_v<<" "<<m_lowpass_w<<" "<<m_lowpass_stuck_pbty<<std::endl;

  if (m_lowpass_stuck_pbty > 1)
  {
    // Remeber to set all forms of stuck probability
    m_lowpass_stuck_pbty = 0;
    m_stuck_pbty = 0;
    return true;
  }
  else
  {
    return false;
  }
}

// Check wether position is reached
bool LocalPlannerPipeline::is_pos_reached(const UGV::UGVModel::State &s, const UGV::UGVModel::State &tgt_state, float reaching_radius)
{
  float2 p_diff = s.p - tgt_state.p;
  if (sqrtf(dot(p_diff,p_diff))<reaching_radius && fabsf(s.v) < 0.5f)
    return true;
  else
    return false;
}

#ifdef PRED_STATE
void LocalPlannerPipeline::update_reference_log(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t)
{
  if(m_cmd_q.empty())
  {
    // queue is empty, just directly add ref into queue
    load_into_queue(ref, curr_t);
  }
  else
  {
    // queue not empty, remove the original content until no more duplicated cmd id
    int diff = ref.ids[0] - m_cmd_q.front().id;
    while(static_cast<int>(m_cmd_q.size()) > diff && !m_cmd_q.empty())
    {
      m_cmd_q.pop_back();
    }
    load_into_queue(ref, curr_t);
  }
}

void LocalPlannerPipeline::load_into_queue(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t)
{
  for (int i=0; i<ref.cols; i++)
  {
    CmdLog tmp;
    tmp.t = curr_t + ros::Duration((i+1)*PSO::PSO_CTRL_DT);

    tmp.id = ref.ids[i];
    tmp.v = ref.data[i*ref.rows];
    tmp.w = ref.data[i*ref.rows + 1];
    tmp.theta = ref.data[i*ref.rows + 2];
    //std::cout<<"id: "<<ref.ids[i]<<", "<<tmp.t<<std::endl;
    m_cmd_q.push_back(tmp);
  }
}
#endif

std::vector<StampedUGVState> LocalPlannerPipeline::make_stamped_reference(int start_idx, const std::vector<UGV::UGVModel::State> & raw_ref)
{
  std::vector<StampedUGVState> stamped_ref;

  int ref_counter = start_idx;
  ros::Time curr_t = m_cycle_start_time;
  ros::Time t_inc = curr_t;
  StampedUGVState tmp;
  for (UGV::UGVModel::State traj_s : raw_ref)
  {
    ref_counter++;
    t_inc = t_inc + ros::Duration(PSO::PSO_CTRL_DT);
    tmp.s = traj_s;
    tmp.id = ref_counter;
    tmp.t = t_inc;
    stamped_ref.push_back(tmp);
  }
  return stamped_ref;
}

void LocalPlannerPipeline::update_norminal_states(const std::vector<StampedUGVState> &stamped_ref)
{
  int next_start_idx = get_next_ref_start_idx();
  for(size_t i=0; i<stamped_ref.size(); i++)
  {
    if(stamped_ref[i].id == next_start_idx)
    {
      m_ref_v = stamped_ref[i].s.v;
      m_ref_w = stamped_ref[i].s.w;
      m_ref_theta = stamped_ref[i].s.theta;
      break;
    }
  }
}

int LocalPlannerPipeline::bool_to_drive_type(bool is_forward)
{
  if (is_forward)
    return cpc_aux_mapping::nf1_task::TYPE_FORWARD;
  else
    return cpc_aux_mapping::nf1_task::TYPE_BACKWARD;
}
