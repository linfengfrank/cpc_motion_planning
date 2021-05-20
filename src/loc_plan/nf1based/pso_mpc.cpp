#include "loc_plan/nf1based/pso_mpc.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/plan_request.h>
#include <cpc_motion_planning/collision_check.h>
#include <std_msgs/Int32MultiArray.h>

PsoMpc::PsoMpc():
  m_goal_received(false),
  cycle_initialized(false),
  m_braking_start_cycle(0)
{
  std::string dp_file_location;
  float var_s, var_theta, step_dt, local_safety_radius;
  int step_num;
  bool use_auto_direction;
  m_nh.param<std::string>("/dp_file_location",dp_file_location,"");
  m_nh.param<float>("/var_s",var_s,2.0);
  m_nh.param<float>("/var_theta",var_theta,1.0);
  m_nh.param<float>("/step_dt",step_dt,1.0);
  m_nh.param<int>("/step_num",step_num,4);
  m_nh.param<bool>("/use_de",m_use_de,false);
  m_nh.param<bool>("/use_auto_direction",use_auto_direction,false);
  m_nh.param<int>("/swarm_size",m_swarm_size,120);
  m_nh.param<int>("/batch_num",m_batch_num,40);
  m_nh.param<int>("/episode_num",m_episode_num,2);
  m_nh.param<float>("/local_safety_radius",local_safety_radius,0.401f);

  m_ref_traj_sub = m_nh.subscribe("/pure_ref_traj",1,&PsoMpc::ref_traj_callback, this);

  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_status_pub = m_nh.advertise<std_msgs::String>("pso_mpc_status_string",1);
  m_force_reset_pub = m_nh.advertise<std_msgs::Int32>("force_reset_state",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &PsoMpc::plan_call_back, this);

  m_pso_planner = new PSO::Planner<PSO_MPC_UGV>(m_swarm_size,m_batch_num,m_episode_num);
  // Init swarm
  m_pso_planner->m_swarm.set_step_dt(step_num, step_dt);
  m_pso_planner->m_swarm.set_var(make_float3(var_s,var_theta,1.0f));
  m_pso_planner->m_eva.m_safety_radius = local_safety_radius;
  m_pso_planner->m_file_location = dp_file_location;
  m_pso_planner->initialize();

  m_ref_v = 0.0f;
  m_ref_w = 0.0f;
  m_ref_theta = 0.0f;

  m_v_err_reset_ctt = 0;
  m_w_err_reset_ctt = 0;
  m_tht_err_reset_ctt = 0;
  //Initialize the control message
  m_ref_msg.rows = 5;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;

  m_create_host_edt = true;
}

PsoMpc::~PsoMpc()
{
  m_pso_planner->release();
  delete m_pso_planner;
}

void PsoMpc::ref_traj_callback(const cpc_motion_planning::ref_data::ConstPtr &msg)
{
  m_goal_received = true;
  m_carrot.p.x = msg->carrot_x;
  m_carrot.p.y = msg->carrot_y;
  m_carrot.theta = msg->carrot_theta;
  set_ref_to_queue(msg, m_ref_queue);
}

void PsoMpc::plan_call_back(const ros::TimerEvent&)
{

  ros::Time curr_t = ros::Time::now();
  cycle_initialized = false;

  cycle_process_based_on_status();

  if (m_status == UGV::START)
    return;

  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;

  ros::Time t_inc = curr_t;
  for (UGV::UGVModel::State traj_s : m_traj)
  {
    ref_counter++;
    t_inc = t_inc + ros::Duration(PSO::PSO_CTRL_DT);
    add_to_ref_msg(m_ref_msg,ref_counter,traj_s,t_inc);

    if (ref_counter == next_ref_start_idx)
    {
      m_ref_v = traj_s.v;
      m_ref_w = traj_s.w;
      m_ref_theta = traj_s.theta;
    }

    cols++;
  }

  //std::cout<<m_ref_theta<<std::endl;

  m_ref_start_idx = next_ref_start_idx;

  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
  update_reference_log(m_ref_msg,curr_t);

  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();
  m_ref_msg.time.clear();
#ifdef SHOW_PC
  plot_trajectory(m_traj);
#endif
  m_plan_cycle++;
}

//=====================================
void PsoMpc::do_start()
{
  if (m_slam_odo_received && m_raw_odo_received && m_received_map && m_goal_received)
  {
    m_ref_theta = get_heading(m_slam_odo);
    m_status = UGV::NORMAL;
    cycle_process_based_on_status();
  }
}
//=====================================
void PsoMpc::do_normal()
{
  cycle_init();
  std::cout<<"NORMAL"<<std::endl;

  while (m_ref_queue.size()>0)
  {
    int id = m_ref_queue.front().idx;
    if (id <= m_ref_start_idx)
      m_ref_queue.pop_front();
    else
      break;
  }

  //check the size of the ref_queue
  if(m_ref_queue.size()<40)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }
  else
  {
    for (int i=0;i<40;i++)
    {
      m_pso_planner->m_eva.m_ref[i] = make_float3(m_ref_queue[i].data[3],m_ref_queue[i].data[4],m_ref_queue[i].data[2]);
    }

  //Planning
    calculate_trajectory<PSO_MPC_UGV>(m_pso_planner, m_traj, m_use_de);
  }

  //Goto: Braking
  if (m_pso_planner->result.collision)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }
  else
  {
    //Goto: Stuck
    if(is_stuck_lowpass(m_pso_planner->m_model.get_ini_state(),m_carrot))
    {
      m_status = UGV::STUCK;
      m_stuck_submode = STUCK_SUB_MODE::FULL_STUCK;
      m_stuck_start_cycle = m_plan_cycle;
      m_full_start_cycle = m_plan_cycle;
    }
  }

}
//=====================================
void PsoMpc::do_stuck()
{
  cycle_init();
  switch (m_stuck_submode)
  {
  case STUCK_SUB_MODE::RECOVER:
    do_recover();
    break;

  case STUCK_SUB_MODE::FULL_STUCK:
    do_full_stuck();
    break;

  }
}
//=====================================
void PsoMpc::do_recover()
{

}
//=====================================
void PsoMpc::do_full_stuck()
{
  std::cout<<"FULL STUCK"<<std::endl;
  //Planning
  m_pso_planner->m_eva.m_stuck = true;
  calculate_trajectory<PSO_MPC_UGV>(m_pso_planner, m_traj);

  //Goto: Braking
  if (m_pso_planner->result.collision)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }

  //Goto: Normal
  if (m_plan_cycle - m_full_start_cycle >= 10)
  {
    std_msgs::Int32 force_reset;
    m_force_reset_pub.publish(force_reset);
    m_status = UGV::NORMAL;
    m_pso_planner->m_eva.m_stuck = false;
  }
}
//=====================================
void PsoMpc::do_emergent()
{
  cycle_init();
}
//=====================================
void PsoMpc::do_braking()
{
  cycle_init();
  std::cout<<"BRAKING"<<std::endl;

  //Planning
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  //Goto: stuck - full stuck
  if (m_plan_cycle - m_braking_start_cycle >= 10)
  {
     m_status = UGV::STUCK;
     m_stuck_submode = STUCK_SUB_MODE::FULL_STUCK;
     m_full_start_cycle = m_plan_cycle;
  }

}
//=====================================
void PsoMpc::do_pos_reached()
{
  cycle_init();
  std::cout<<"POS_REACHED"<<std::endl;
}
//=====================================
void PsoMpc::do_fully_reached()
{
  cycle_init();
  std::cout<<"FULLY_REACHED"<<std::endl;
}
//=====================================
void PsoMpc::do_dropoff()
{
  cycle_init();
  std::cout<<"DROPOFF"<<std::endl;
}
//=====================================
void PsoMpc::cycle_init()
{
  if (cycle_initialized)
    return;

  cycle_initialized = true;
  bool is_heading_ref;
  float psi = select_mes_ref_heading(is_heading_ref,get_heading(m_slam_odo), m_ref_theta, m_tht_err_reset_ctt, 0.25f);

  UGV::UGVModel::State s = predict_state(m_slam_odo,psi,m_ref_start_idx,is_heading_ref);

  s.v = select_mes_ref(m_raw_odo.twist.twist.linear.x, m_ref_v, m_v_err_reset_ctt);
  s.w = select_mes_ref(m_raw_odo.twist.twist.angular.z, m_ref_w, m_w_err_reset_ctt);

  m_pso_planner->m_model.set_ini_state(s);
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());
  //m_traj = m_pso_planner->generate_trajectory();

  //Publish the ref status string for loging
  std_msgs::String msg;
  std::stringstream ss;
  ss << "STT: " << m_status<<", CST:"<< m_pso_planner->result.best_cost<<", COL:"<<m_pso_planner->result.collision<<std::endl;
  msg.data = ss.str();
  m_status_pub.publish(msg);
}
