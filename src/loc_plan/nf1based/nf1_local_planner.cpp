#include "loc_plan/nf1based/nf1_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/plan_request.h>
#include <cpc_motion_planning/collision_check.h>

NF1LocalPlanner::NF1LocalPlanner():
  m_goal_received(false),
  m_mid_goal_received(false),
  cycle_initialized(false),
  m_braking_start_cycle(0),
  m_nf1_map(nullptr)
{
  m_goal_sub = m_nh.subscribe("/set_global_goal",1,&NF1LocalPlanner::goal_call_back, this);
  m_mid_goal_sub = m_nh.subscribe("/set_middle_goal",1,&NF1LocalPlanner::mid_goal_call_back, this);
  m_nf1_sub = m_nh.subscribe("/nf1",1,&NF1LocalPlanner::nf1_call_back, this);
  m_line_tgt_sub = m_nh.subscribe("/line_target",1,&NF1LocalPlanner::line_target_call_back,this);
  m_hybrid_path_sub = m_nh.subscribe("/hybrid_path",1,&NF1LocalPlanner::hybrid_path_call_back,this);

  m_collision_check_client = m_nh.serviceClient<cpc_motion_planning::collision_check>("collision_check");
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_status_pub = m_nh.advertise<std_msgs::String>("ref_status_string",1);
  m_tgt_reached_pub = m_nh.advertise<std_msgs::Int32>("target_reached",1);
  m_stuck_plan_request_pub = m_nh.advertise<cpc_motion_planning::plan_request>("plan_request",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &NF1LocalPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<SIMPLE_UGV>(120,50,3);
  m_pso_planner->initialize();


  m_mid_goal=make_float2(0,0);

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
}

NF1LocalPlanner::~NF1LocalPlanner()
{
  m_pso_planner->release();
  delete m_pso_planner;
}

void NF1LocalPlanner::nf1_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  m_goal_received = true;
  if (m_nf1_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 m_nf1_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_nf1_map = new NF1MapDT(origin,msg->width,m_nf1_map_size);
    m_nf1_map->setup_device();
  }
  else
  {
    m_nf1_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_nf1_map->m_grid_step = msg->width;
  }
  CUDA_MEMCPY_H2D(m_nf1_map->m_nf1_map,msg->payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));
  m_pso_planner->m_eva.m_nf1_map = *m_nf1_map;
  m_pso_planner->m_eva.m_nf1_received = true;
}

void NF1LocalPlanner::plan_call_back(const ros::TimerEvent&)
{

  ros::Time curr_t = ros::Time::now();
  cycle_initialized = false;

  cycle_process_based_on_status();

  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;


  for (UGV::UGVModel::State traj_s : m_traj)
  {
    ref_counter++;
    add_to_ref_msg(m_ref_msg,ref_counter,traj_s);

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
#ifdef SHOW_PC
  plot_trajectory(m_traj);
#endif
  m_plan_cycle++;
}

void NF1LocalPlanner::line_target_call_back(const cpc_motion_planning::line_target::ConstPtr &msg)
{
  m_goal_received = true;
  m_goal.s.p.x = msg->target_pose.pose.position.x;
  m_goal.s.p.y = msg->target_pose.pose.position.y;

  double phi,theta,psi;

  tf::Quaternion q( msg->target_pose.pose.orientation.x,
                    msg->target_pose.pose.orientation.y,
                    msg->target_pose.pose.orientation.z,
                    msg->target_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);

  m_goal.s.theta = psi;
  m_goal.id++;

  m_goal.do_turning = msg->do_turning;
  m_goal.reaching_radius = msg->reaching_radius;
}

void NF1LocalPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_goal_received = true;
  m_goal.s.p.x = msg->pose.position.x;
  m_goal.s.p.y = msg->pose.position.y;

  double phi,theta,psi;

  tf::Quaternion q( msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);


  m_goal.s.theta = psi;
  m_goal.id++;

  m_goal.do_turning = true;
  m_goal.reaching_radius = 1.0f;
}

void NF1LocalPlanner::mid_goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_mid_goal_received = true;
  m_mid_goal.x = msg->pose.position.x;
  m_mid_goal.y = msg->pose.position.y;
}

void NF1LocalPlanner::hybrid_path_call_back(const cpc_motion_planning::path::ConstPtr &msg)
{
  if (msg->request_ctt != m_plan_request_cycle)
    return;
  m_stuck_recover_path = *msg;
  m_recover_planner.set_path_cell(m_stuck_recover_path);
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
  m_pso_planner->m_eva.m_pure_turning = false;
  m_pso_planner->m_eva.setTarget(m_goal);
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj);

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
    if(is_stuck(m_traj,m_goal.s) || is_stuck_lowpass(m_pso_planner->m_model.get_ini_state(), m_goal.s))
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
      if(m_goal.do_turning)
      {
        m_status = UGV::POS_REACHED;
      }
      else
      {
        std_msgs::Int32 reach_msg;
        reach_msg.data = m_goal.id;
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
  // overall time out
  if (m_plan_cycle - m_stuck_start_cycle >= 50)
  {
    m_status = UGV::NORMAL;
  }

  // have not received response yet
  if (m_stuck_recover_path.request_ctt != m_plan_request_cycle)
  {
    std::cout<<"WAIT FOR HYBRID A"<<std::endl;
    full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());
    return;
  }

  std::cout<<"RECOVER"<<std::endl;
  if (m_stuck_recover_path.actions.size() == 0)
  {
    // the response is empty, go to the full stuck sub mode
    full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());
    m_stuck_submode = STUCK_SUB_MODE::FULL_STUCK;
    m_full_start_cycle = m_plan_cycle;
    return;
  }

  // do the recover path following
  bool finished = m_recover_planner.calculate_trajectory(m_pso_planner->m_model.get_ini_state(),
                                                         m_edt_map,
                                                         m_traj);

  if (m_recover_planner.should_braking())
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
    return;
  }

  if(finished)
  {
    m_status = UGV::NORMAL;
    return;
  }

  // if still stuck
  if(is_stuck(m_traj,m_goal.s) || is_stuck_lowpass(m_pso_planner->m_model.get_ini_state(), m_goal.s))
  {
    // if the ref path has collision, just go back to the normal mode
    if (check_collision_from_host_edt(m_recover_planner.get_collision_checking_path()))
    {
      m_status = UGV::NORMAL;
    }
    else
    {
      m_stuck_submode = STUCK_SUB_MODE::FULL_STUCK;
      m_full_start_cycle = m_plan_cycle;
    }
  }
}
//=====================================
void NF1LocalPlanner::do_full_stuck()
{
  std::cout<<"FULL STUCK"<<std::endl;
  //Planning
  m_pso_planner->m_eva.m_stuck = true;
  m_pso_planner->m_eva.setTarget(m_goal);
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj);

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

  // Planning
  m_pso_planner->m_eva.m_pure_turning = true;
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj);
  if(is_heading_reached(m_pso_planner->m_model.get_ini_state(),m_goal.s))
  {
    std_msgs::Int32 reach_msg;
    reach_msg.data = m_goal.id;
    m_tgt_reached_pub.publish(reach_msg);
  }

  //Goto: Normal (New target)
  if (m_goal.id != m_pso_planner->m_eva.m_goal.id)
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
  if (m_goal.id != m_pso_planner->m_eva.m_goal.id)
  {
    m_status = UGV::NORMAL;
  }
}
//=====================================
void NF1LocalPlanner::cycle_init()
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
  m_traj = m_pso_planner->generate_trajectory();

  //Publish the ref status string for loging
  std_msgs::String msg;
  std::stringstream ss;
  ss << "STT: " << m_status<<", CST:"<< m_pso_planner->result.best_cost<<", COL:"<<m_pso_planner->result.collision<<std::endl;
  msg.data = ss.str();
  m_status_pub.publish(msg);
}
