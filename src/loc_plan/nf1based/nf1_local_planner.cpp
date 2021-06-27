#include "loc_plan/nf1based/nf1_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/plan_request.h>
#include <cpc_motion_planning/collision_check.h>
#include <std_msgs/Int32MultiArray.h>

NF1LocalPlanner::NF1LocalPlanner():
  m_goal_received(false),
  m_task_is_new(false),
  cycle_initialized(false),
  m_braking_start_cycle(0),
  m_nf1_map(nullptr)
{
  std::string dp_file_location;
  float var_s, var_theta, step_dt, local_safety_radius, max_speed;
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
  m_nh.param<float>("/max_speed", max_speed, 1.0f);

  m_nf1_sub = m_nh.subscribe("/nf1",1,&NF1LocalPlanner::nf1_call_back, this);

  m_collision_check_client = m_nh.serviceClient<cpc_motion_planning::collision_check>("collision_check");
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_status_pub = m_nh.advertise<std_msgs::String>("ref_status_string",1);
  m_tgt_reached_pub = m_nh.advertise<std_msgs::Int32MultiArray>("target_reached",1);
  m_stuck_plan_request_pub = m_nh.advertise<cpc_motion_planning::plan_request>("plan_request",1);
  m_drive_dir_pub = m_nh.advertise<std_msgs::Int32>("drive_dir",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &NF1LocalPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<SIMPLE_UGV>(m_swarm_size,m_batch_num,m_episode_num);
  // Init swarm
  m_pso_planner->m_swarm.set_step_dt(step_num, step_dt);
  m_pso_planner->m_swarm.set_var(make_float3(var_s,var_theta,1.0f));
  m_pso_planner->m_eva.m_using_auto_direction = use_auto_direction;
  m_pso_planner->m_eva.m_safety_radius = local_safety_radius;
  m_pso_planner->m_eva.m_max_speed = max_speed;
  m_pso_planner->m_file_location = dp_file_location;
  m_pso_planner->initialize();
  m_recover_planner.init_swarm(step_num, step_dt, var_s, var_theta, dp_file_location);

  m_ref_v = 0.0f;
  m_ref_w = 0.0f;
  m_ref_theta = 0.0f;

  m_v_err_reset_ctt = 0;
  m_w_err_reset_ctt = 0;
  m_tht_err_reset_ctt = 0;
  m_stuck_recover_path.request_ctt = -1;
  //Initialize the control message
  m_ref_msg.rows = 5;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;

  m_create_host_edt = true;
  m_min_dist = 0.0f;
}

NF1LocalPlanner::~NF1LocalPlanner()
{
  m_pso_planner->release();
  delete m_pso_planner;
}

void NF1LocalPlanner::nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg)
{
  m_goal_received = true;
  // setup the map
  if (m_nf1_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->nf1.x_origin,msg->nf1.y_origin,msg->nf1.z_origin);
    int3 m_nf1_map_size = make_int3(msg->nf1.x_size,msg->nf1.y_size,msg->nf1.z_size);
    m_nf1_map = new NF1MapDT(origin,msg->nf1.width,m_nf1_map_size);
    m_nf1_map->setup_device();
  }
  else
  {
    m_nf1_map->m_origin = CUDA_GEO::pos(msg->nf1.x_origin,msg->nf1.y_origin,msg->nf1.z_origin);
    m_nf1_map->m_grid_step = msg->nf1.width;
  }
  CUDA_MEMCPY_H2D(m_nf1_map->m_nf1_map,msg->nf1.payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));
  m_pso_planner->m_eva.m_nf1_map = *m_nf1_map;
  m_pso_planner->m_eva.m_nf1_received = true;

  // setup the drive type
  if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_FORWARD)
  {
    m_pso_planner->m_eva.m_pure_turning = false;
    m_pso_planner->m_eva.is_forward = true;
  }
  else if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_BACKWARD)
  {
    m_pso_planner->m_eva.m_pure_turning = false;
    m_pso_planner->m_eva.is_forward = false;
  }
  else if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_ROTATE)
  {
    m_pso_planner->m_eva.m_pure_turning = true;
    m_pso_planner->m_eva.is_forward = true;
  }

  // setup the goal and carrot
  m_goal.s.p.x = msg->goal_x;
  m_goal.s.p.y = msg->goal_y;
  m_goal.s.theta = msg->goal_theta;
  m_goal.path_id = msg->path_id;
  m_goal.act_id = msg->act_id;
  m_goal.reaching_radius = 0.5f;

  m_carrot.p.x = msg->carrot_x;
  m_carrot.p.y = msg->carrot_y;
  m_carrot.theta = msg->carrot_theta;

  if (!check_tgt_is_same(m_goal, m_pso_planner->m_eva.m_goal))
  {
    m_task_is_new = true;
  }

  m_pso_planner->m_eva.setTarget(m_goal);
}

void NF1LocalPlanner::plan_call_back(const ros::TimerEvent&)
{

  ros::Time curr_t = ros::Time::now();
  cycle_initialized = false;

  cycle_process_based_on_status();

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
void NF1LocalPlanner::do_start()
{
  if (m_slam_odo_received && m_raw_odo_received && m_received_map && m_goal_received)
  {
    m_status = UGV::NORMAL;
  }
  else
  {
    if (m_slam_odo_received)
      m_ref_theta = get_heading(m_slam_odo);
  }
}
//=====================================
void NF1LocalPlanner::do_normal()
{
  cycle_init();
  std::cout<<"NORMAL"<<std::endl;

  //Planning
  float var_s = 2.0f * (m_min_dist);
  if (var_s < 0.4f) var_s = 0.4f;
  if (var_s > 3.0f) var_s = 3.0f;


  m_pso_planner->m_swarm.set_var_s(var_s);
  m_task_is_new = false;
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj, m_use_de);

  m_min_dist = m_pso_planner->result.best_loc.min_dist;


  //Update the drive direction
  std_msgs::Int32 drive_dir;
  if (m_pso_planner->result.best_loc[0].z > 0)
    drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_FORWARD;
  else
    drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_BACKWARD;

  m_drive_dir_pub.publish(drive_dir);

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
    if(is_stuck(m_traj,m_carrot) || is_stuck_lowpass(m_pso_planner->m_model.get_ini_state(),m_carrot))
    {
      m_status = UGV::STUCK;
      m_stuck_submode = STUCK_SUB_MODE::FULL_STUCK;
      m_stuck_start_cycle = m_plan_cycle;
      m_plan_request_cycle = m_plan_cycle;
      m_full_start_cycle = m_plan_cycle;

      // Prepare and send the request message
      cpc_motion_planning::plan_request rq_msg;
      rq_msg.request_ctt = m_plan_cycle;
      rq_msg.start_x = m_pso_planner->m_model.get_ini_state().p.x;
      rq_msg.start_y = m_pso_planner->m_model.get_ini_state().p.y;
      rq_msg.start_theta = m_pso_planner->m_model.get_ini_state().theta;
      rq_msg.goal_x = m_goal.s.p.x;
      rq_msg.goal_y = m_goal.s.p.y;
      rq_msg.goal_theta = m_goal.s.theta;
      m_stuck_plan_request_pub.publish(rq_msg);
    }

    //Goto: Pos_reached
    if(is_pos_reached(m_pso_planner->m_model.get_ini_state(),m_goal.s,m_goal.reaching_radius))
    {
        float2 carrot_diff = m_pso_planner->m_model.get_ini_state().p - m_carrot.p;
        if (sqrtf(dot(carrot_diff,carrot_diff))<m_goal.reaching_radius)
        {
          m_status = UGV::POS_REACHED;
          std_msgs::Int32MultiArray reach_msg;
          reach_msg.data.push_back(m_goal.path_id);
          reach_msg.data.push_back(m_goal.act_id);
          m_tgt_reached_pub.publish(reach_msg);
        }
    }
  }
}
//=====================================
void NF1LocalPlanner::do_stuck()
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
void NF1LocalPlanner::do_recover()
{

}
//=====================================
void NF1LocalPlanner::do_full_stuck()
{
  std::cout<<"FULL STUCK"<<std::endl;
  //Planning
  m_pso_planner->m_eva.m_stuck = true;
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj);

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
    m_status = UGV::NORMAL;
    m_pso_planner->m_eva.m_stuck = false;
  }
}
//=====================================
void NF1LocalPlanner::do_emergent()
{
  cycle_init();
}
//=====================================
void NF1LocalPlanner::do_braking()
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
void NF1LocalPlanner::do_pos_reached()
{
  cycle_init();
  std::cout<<"POS_REACHED"<<std::endl;

  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  //Goto: Normal (New target)
  if (m_task_is_new)
  {
    m_status = UGV::NORMAL;
  }
}
//=====================================
void NF1LocalPlanner::do_fully_reached()
{
  cycle_init();
  std::cout<<"FULLY_REACHED"<<std::endl;

  // Planing
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  //Go to the dropoff state
  m_status = UGV::DROPOFF;
}
//=====================================
void NF1LocalPlanner::do_dropoff()
{
  cycle_init();
  std::cout<<"DROPOFF"<<std::endl;

  // Planing
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  //Goto: Normal (New target)
  if (m_task_is_new)
  {
    m_status = UGV::NORMAL;
  }
}
//=====================================
void NF1LocalPlanner::cycle_init()
{
  if (cycle_initialized)
    return;

//  float2 diff = m_goal.s.p - m_carrot.p;
//  if (sqrtf(dot(diff,diff)) <= 2*m_edt_map->m_grid_step)
//    m_pso_planner->m_eva.m_accurate_reaching = true;
//  else
//    m_pso_planner->m_eva.m_accurate_reaching = false;

  cycle_initialized = true;
  bool is_heading_ref;
  float psi = select_mes_ref_heading(is_heading_ref,get_heading(m_slam_odo), m_ref_theta, m_tht_err_reset_ctt, 0.25f);

  UGV::UGVModel::State s = predict_state(m_slam_odo,psi,m_ref_start_idx,is_heading_ref);

  s.v = select_mes_ref(m_raw_odo.twist.twist.linear.x, m_ref_v, m_v_err_reset_ctt);
  s.w = select_mes_ref(m_raw_odo.twist.twist.angular.z, m_ref_w, m_w_err_reset_ctt);

  m_pso_planner->m_model.set_ini_state(s);
  m_traj = m_pso_planner->generate_trajectory();

  //Publish the ref status string for loging
  std_msgs::String msg;
  std::stringstream ss;
  ss << "STT: " << m_status<<", CST:"<< m_pso_planner->result.best_cost<<", COL:"<<m_pso_planner->result.collision<<std::endl;
  msg.data = ss.str();
  m_status_pub.publish(msg);
}
