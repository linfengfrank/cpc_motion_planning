#include "loc_plan/tebbased/teb_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/plan_request.h>
#include <cpc_motion_planning/collision_check.h>
#include <std_msgs/Int32MultiArray.h>
#include <chrono>
using namespace teb_local_planner;

TEBLocalPlanner::TEBLocalPlanner():
  m_goal_received(false),
  m_task_is_new(false),
  cycle_initialized(false),
  m_braking_start_cycle(0),
  m_nf1_map(nullptr)
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

  m_nf1_sub = m_nh.subscribe("/nf1",1,&TEBLocalPlanner::nf1_call_back, this);

  m_collision_check_client = m_nh.serviceClient<cpc_motion_planning::collision_check>("collision_check");
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_status_pub = m_nh.advertise<std_msgs::String>("ref_status_string",1);
  m_tgt_reached_pub = m_nh.advertise<std_msgs::Int32MultiArray>("target_reached",1);
  m_stuck_plan_request_pub = m_nh.advertise<cpc_motion_planning::plan_request>("plan_request",1);
  m_drive_dir_pub = m_nh.advertise<std_msgs::Int32>("drive_dir",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &TEBLocalPlanner::plan_call_back, this);

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

  //-----------
  m_cfg.loadRosParamFromNodeHandle(m_nh);
  m_cfg.map_frame = "world";

  m_visualization = TebVisualizationPtr(new TebVisualization(m_nh, m_cfg));

  // create robot footprint/contour model for optimization
  m_planner = HomotopyClassPlannerPtr(new HomotopyClassPlanner(m_cfg, m_visualization));

}

TEBLocalPlanner::~TEBLocalPlanner()
{

}

void TEBLocalPlanner::nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg)
{
  m_goal_received = true;
  // setup the map
  if (m_nf1_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->nf1.x_origin,msg->nf1.y_origin,msg->nf1.z_origin);
    int3 m_nf1_map_size = make_int3(msg->nf1.x_size,msg->nf1.y_size,msg->nf1.z_size);
    m_nf1_map = new NF1MapDT(origin,msg->nf1.width,m_nf1_map_size);
    m_nf1_map->m_create_host_cpy = true;
    m_nf1_map->setup_device();
  }
  else
  {
    m_nf1_map->m_origin = CUDA_GEO::pos(msg->nf1.x_origin,msg->nf1.y_origin,msg->nf1.z_origin);
    m_nf1_map->m_grid_step = msg->nf1.width;
  }
  CUDA_MEMCPY_H2D(m_nf1_map->m_nf1_map,msg->nf1.payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));
  memcpy(m_nf1_map->m_hst_map,msg->nf1.payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));

  // setup the drive type

  // setup the drive type
  if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_FORWARD)
  {
    m_planner->m_is_forward = true;
  }
  else if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_BACKWARD)
  {
    m_planner->m_is_forward = false;
  }
  else if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_ROTATE)
  {
    m_planner->m_is_forward = true;
  }

  // setup the goal and carrot
  UGV::NF1Evaluator::Target tmp_goal;
  tmp_goal.s.p.x = msg->goal_x;
  tmp_goal.s.p.y = msg->goal_y;
  tmp_goal.s.theta = msg->goal_theta;
  tmp_goal.path_id = msg->path_id;
  tmp_goal.act_id = msg->act_id;
  tmp_goal.reaching_radius = 0.5f;

  if(!check_tgt_is_same(m_goal, tmp_goal))
  {
    m_goal = tmp_goal;
    m_task_is_new = true;
  }

  m_carrot.p.x = msg->carrot_x;
  m_carrot.p.y = msg->carrot_y;
  m_carrot.theta = msg->carrot_theta;

}

void TEBLocalPlanner::plan_call_back(const ros::TimerEvent&)
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
void TEBLocalPlanner::do_start()
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
void TEBLocalPlanner::do_normal()
{

  auto start_time = std::chrono::steady_clock::now();
  cycle_init();
  std::cout<<"NORMAL"<<std::endl;

  //Planning
  m_task_is_new = false;

  PoseSE2 robot_pose(m_ini_state.p.x, m_ini_state.p.y, m_ini_state.theta);
  PoseSE2 robot_goal(m_carrot.p.x, m_carrot.p.y, m_carrot.theta);
  geometry_msgs::Twist robot_vel;
  robot_vel.linear.x = m_ini_state.v;
  robot_vel.angular.z = m_ini_state.w;

  if (!m_planner->get_edt_map())
    m_planner->set_edt_map(m_edt_map);

  std::vector<double3> init = get_init_guess();
  //std::cout<<init.size()<<std::endl;
  m_planner->set_init_plan(init);

  bool success = m_planner->plan(robot_pose, robot_goal,m_cfg.trajectory.feasibility_check_no_poses, &robot_vel, m_cfg.goal_tolerance.free_goal_vel);

 auto end_time = std::chrono::steady_clock::now();
      std::cout << "Local planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms"<< std::endl;

  std_msgs::Int32 drive_dir;
  if (m_planner->m_is_forward)
    drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_FORWARD;
  else
    drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_BACKWARD;

  m_drive_dir_pub.publish(drive_dir);

  if (!success)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
    return;
  }

  bool feasible = m_planner->isTrajectoryFeasible();
  if(!feasible)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
    return;
  }

  std::vector<Reference> ref;
  if (m_planner->bestTeb() && m_planner->bestTeb()->get_reference(4,0.05, ref))
  {
    m_traj.clear();
    for (Reference r : ref)
    {
      UGV::UGVModel::State s;
      s.p.x = r.pose.x();
      s.p.y = r.pose.y();
      s.v = r.vx;
      s.theta = r.pose.theta();
      s.w = r.omega;
      m_traj.push_back(s);
    }

    if(is_pos_reached(m_ini_state,m_goal.s,m_goal.reaching_radius))
    {
        float2 carrot_diff = m_ini_state.p - m_carrot.p;
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
  else
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
    return;
  }
}
//=====================================
void TEBLocalPlanner::do_stuck()
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
void TEBLocalPlanner::do_recover()
{

}
//=====================================
void TEBLocalPlanner::do_full_stuck()
{
  std::cout<<"FULL STUCK"<<std::endl;
}
//=====================================
void TEBLocalPlanner::do_emergent()
{
  cycle_init();
}
//=====================================
void TEBLocalPlanner::do_braking()
{
  cycle_init();
  std::cout<<"BRAKING"<<std::endl;

  //Planning
  full_stop_trajectory(m_traj,m_ini_state);

  //Goto: stuck - full stuck
  if (m_plan_cycle - m_braking_start_cycle >= 10)
  {
     m_status = UGV::NORMAL;
     m_full_start_cycle = m_plan_cycle;
  }

}
//=====================================
void TEBLocalPlanner::do_pos_reached()
{
  cycle_init();
  std::cout<<"POS_REACHED"<<std::endl;

  full_stop_trajectory(m_traj,m_ini_state);

  //Goto: Normal (New target)
  if (m_task_is_new)
  {
    m_status = UGV::NORMAL;
  }
}
//=====================================
void TEBLocalPlanner::do_fully_reached()
{
  cycle_init();
  std::cout<<"FULLY_REACHED"<<std::endl;

  // Planing
  full_stop_trajectory(m_traj,m_ini_state);

  //Go to the dropoff state
  m_status = UGV::DROPOFF;
}
//=====================================
void TEBLocalPlanner::do_dropoff()
{
  cycle_init();
  std::cout<<"DROPOFF"<<std::endl;

  // Planing
  full_stop_trajectory(m_traj,m_ini_state);

  //Goto: Normal (New target)
  if (m_task_is_new)
  {
    m_status = UGV::NORMAL;
  }
}
//=====================================
void TEBLocalPlanner::cycle_init()
{
  if (cycle_initialized)
    return;

  cycle_initialized = true;
  bool is_heading_ref;
  float psi = select_mes_ref_heading(is_heading_ref,get_heading(m_slam_odo), m_ref_theta, m_tht_err_reset_ctt, 0.25f);

  UGV::UGVModel::State s = predict_state(m_slam_odo,psi,m_ref_start_idx,is_heading_ref);

  s.v = select_mes_ref(m_raw_odo.twist.twist.linear.x, m_ref_v, m_v_err_reset_ctt);
  s.w = select_mes_ref(m_raw_odo.twist.twist.angular.z, m_ref_w, m_w_err_reset_ctt);

  m_ini_state = s;

  full_stop_trajectory(m_traj,m_ini_state);

}
//---
std::vector<double3> TEBLocalPlanner::get_init_guess()
{
  std::vector<double3> pre_guess, guess;
  // the initial pose
  pre_guess.push_back(make_double3(m_ini_state.p.x,
                                m_ini_state.p.y,
                                m_ini_state.theta));
  CUDA_GEO::pos p;
  p.x = m_ini_state.p.x;
  p.y = m_ini_state.p.y;
  CUDA_GEO::coord c = m_nf1_map->pos2coord(p);
  CUDA_GEO::coord bc;
  // First calculate the positions
  while(1)
  {
    if(get_smallest_child(c,bc))
    {
      c = bc;
      p = m_nf1_map->coord2pos(c);
      pre_guess.push_back(make_double3(p.x,p.y,0));
    }
    else
    {
      break;
    }
  }
  if (pre_guess.size()>1)
    pre_guess.pop_back();

  // the goal pose
  pre_guess.push_back(make_double3(m_carrot.p.x,
                                m_carrot.p.y,
                                m_carrot.theta));

   std::vector<size_t> split_idx = split_merge(pre_guess, 0.1);

   for(size_t i=0; i<split_idx.size(); i++)
   {
     guess.push_back(pre_guess[split_idx[i]]);
   }


  // Then calculate the angle
  for (int i=1; i < static_cast<int>(guess.size())-1; i++)
  {
    // get yaw from the orientation of the distance vector between pose_{i+1} and pose_{i}
    double dx = guess[i+1].x - guess[i].x;
    double dy = guess[i+1].y - guess[i].y;
    double yaw = std::atan2(dy,dx);
    guess[i].z = yaw;
  }
  return guess;
}

bool TEBLocalPlanner::get_smallest_child(const CUDA_GEO::coord &c, CUDA_GEO::coord& bc)
{
  CUDA_GEO::coord g;
  bool found = false;
  float best_cost = m_nf1_map->getCostHst(c.x,c.y,0);
  for (int i=-1;i<=1;i++)
  {
    for(int j=-1;j<=1;j++)
    {
      if(i==0 && j==0)
        continue;

      g.x = c.x+i;
      g.y = c.y+j;
      float cost = m_nf1_map->getCostHst(g.x,g.y,0);
      if (best_cost > cost)
      {
        found = true;
        best_cost = cost;
        bc = g;
      }
    }
  }
  return found;
}
