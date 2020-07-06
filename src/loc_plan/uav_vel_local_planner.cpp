#include <loc_plan/uav_vel_local_planner.h>
#include "tf/tf.h"
#include <chrono>
#include <std_srvs/Empty.h>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

UAVVelMotionPlanner::UAVVelMotionPlanner():
  UAVLocalMotionPlanner(),
  m_goal_received(false)
{
  m_goal_sub = m_nh.subscribe("/mid_layer/goal",1,&UAVVelMotionPlanner::goal_call_back, this);

  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UAVVelMotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<VEL_UAV>(150,30,1);
  m_pso_planner->initialize();

  m_emergent_planner = new PSO::Planner<EMERGENT_UAV>(50,20,1);
  m_emergent_planner->initialize();
  m_emergent_planner->m_ctrl_dev.set_limit(make_float2(5,2), make_float2(4,2), make_float2(5,5));
  m_emergent_planner->m_ctrl_host.set_limit(make_float2(5,2), make_float2(4,2), make_float2(5,5));


  //Initialize the control message
  m_ref_msg.rows = 12;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;
}

UAVVelMotionPlanner::~UAVVelMotionPlanner()
{
  m_pso_planner->release();
  delete m_pso_planner;

  m_emergent_planner->release();
  delete m_emergent_planner;
}

void UAVVelMotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  cycle_init();
  cycle_process_based_on_status();
  if (m_fly_status <= UAV::AT_GROUND)
    return;

  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;
  float t = 0.0f;
  int i = 0;

  for (UAV::UAVModel::State traj_s : m_traj)
  {
    t += PSO::PSO_CTRL_DT;
    JLT::State yaw_state = m_yaw_traj[i++];

    ref_counter++;
    add_to_ref_msg(m_ref_msg,ref_counter,traj_s,yaw_state);

    if (ref_counter == next_ref_start_idx)
    {
      m_curr_ref = traj_s;
      m_head_sov.set_yaw_state(yaw_state);
    }

    cols++;
  }

  m_ref_start_idx = next_ref_start_idx;
  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();

  m_plan_cycle++;
#ifdef SHOWPC
  plot_trajectory(m_traj);
#endif
}

void UAVVelMotionPlanner::goal_call_back(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  m_goal_received = true;
  m_goal.v.x = msg->twist.linear.x;
  m_goal.v.y = msg->twist.linear.y;
  m_goal.v.z = msg->twist.linear.z;
  m_pso_planner->m_eva.setTarget(m_goal);
}

void UAVVelMotionPlanner::do_at_ground()
{
  if (m_pose_received && m_received_map && m_goal_received)
  {
    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("engage", eReq, eRes);
    m_fly_status = UAV::TAKING_OFF;
  }
}
void UAVVelMotionPlanner::do_taking_off()
{
  auto start = std::chrono::steady_clock::now();
  calculate_trajectory<VEL_UAV>(m_pso_planner,m_traj);
  if (m_curr_ref.p.z >= 1.8f && fabsf(m_curr_ref.v.z)<0.3f)
  {
    m_goal.oa = m_goal_received;
    m_e_goal.oa = m_goal_received;
    m_pso_planner->m_eva.setTarget(m_goal);
    m_fly_status = UAV::IN_AIR;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;
}
void UAVVelMotionPlanner::do_in_air()
{
  auto start = std::chrono::steady_clock::now();
  calculate_trajectory<VEL_UAV>(m_pso_planner,m_traj);
  if (m_pso_planner->result.collision)
  {
    m_fly_status = UAV::EMERGENT;
    cycle_process_based_on_status();
  }
  else
  {
    m_head_sov.cal_yaw_target_from_vel(m_goal.v,m_curr_ref);
    m_yaw_traj = m_head_sov.generate_yaw_traj();
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;

}
void UAVVelMotionPlanner::do_emergent()
{
  auto start = std::chrono::steady_clock::now();
  calculate_trajectory<EMERGENT_UAV>(m_emergent_planner,m_traj);
  if(m_emergent_planner->result.collision)
  {
    m_fly_status = UAV::BRAKING;
    m_start_braking_cycle = m_plan_cycle;
    m_curr_ref = odom2state(m_pose);
    cycle_process_based_on_status();
  }
  else
  {
    m_fly_status = UAV::IN_AIR;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (emergent): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_emergent_planner->result.best_cost
            << ", collision: " << m_emergent_planner->result.collision<<std::endl;

}

void UAVVelMotionPlanner::do_braking()
{
  auto start = std::chrono::steady_clock::now();
  m_rep_filed.generate_repulse_traj(m_traj, *m_edt_map, m_curr_ref);
  if (m_plan_cycle - m_start_braking_cycle > 20)
  {
    m_fly_status = UAV::IN_AIR;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (braking): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_emergent_planner->result.best_cost
            << ", collision: " << m_emergent_planner->result.collision<<std::endl;
}

void UAVVelMotionPlanner::do_stuck()
{

}

void UAVVelMotionPlanner::cycle_init()
{
  // Set the initial states for all planners
  m_pso_planner->m_model.set_ini_state(m_curr_ref);
  m_pso_planner->m_eva.m_curr_yaw = m_head_sov.get_yaw();
  m_pso_planner->m_eva.m_curr_pos = m_curr_ref.p;

  m_emergent_planner->m_model.set_ini_state(m_curr_ref);
  m_emergent_planner->m_eva.m_curr_yaw = m_head_sov.get_yaw();
  m_emergent_planner->m_eva.m_curr_pos = m_curr_ref.p;

  m_e_goal.s = m_curr_ref;
  m_emergent_planner->m_eva.setTarget(m_e_goal);

  // Construct default trajectories for pos and yaw
  switch (m_fly_status) {
  case UAV::AT_GROUND:
  {
    generate_static_traj(m_traj, m_curr_ref);
    break;
  }
  case UAV::TAKING_OFF:
  case UAV::IN_AIR:
  {
     m_traj = m_pso_planner->generate_trajectory();
    break;
  }
  case UAV::EMERGENT:
  {
    m_traj = m_emergent_planner->generate_trajectory();
    break;
  }
  case UAV::BRAKING:
  {
    generate_static_traj(m_traj, m_curr_ref);
    break;
  }
  default:
    break;
  }

  m_yaw_traj = m_head_sov.generate_yaw_traj();
}
