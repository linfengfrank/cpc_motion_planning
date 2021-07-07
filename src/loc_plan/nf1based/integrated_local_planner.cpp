#include "loc_plan/nf1based/integrated_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/plan_request.h>
#include <cpc_motion_planning/collision_check.h>
#include <std_msgs/Int32MultiArray.h>

IntLocalPlanner::IntLocalPlanner():
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
  m_nh.param<bool>("/use_simple_filter",m_use_simple_filter,true);

  m_nf1_sub = m_nh.subscribe("/nf1",1,&IntLocalPlanner::nf1_call_back, this);

  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_status_pub = m_nh.advertise<std_msgs::String>("ref_status_string",1);
  m_tgt_reached_pub = m_nh.advertise<std_msgs::Int32MultiArray>("target_reached",1);
  m_drive_dir_pub = m_nh.advertise<std_msgs::Int32>("drive_dir",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &IntLocalPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<SIMPLE_UGV>(m_swarm_size,m_batch_num,m_episode_num);
  // Init swarm
  m_pso_planner->m_swarm.set_step_dt(step_num, step_dt);
  m_pso_planner->m_swarm.set_var(make_float3(var_s,var_theta,1.0f));
  m_pso_planner->m_eva.m_using_auto_direction = use_auto_direction;
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
  //-----------
  m_cfg.loadRosParamFromNodeHandle(m_nh);
  m_cfg.map_frame = "world";

  m_visualization = teb::TebVisualizationPtr(new teb::TebVisualization(m_nh, m_cfg));

  // create robot footprint/contour model for optimization
  m_teb_planner = teb::HomotopyClassPlannerPtr(new teb::HomotopyClassPlanner(m_cfg, m_visualization));

  //==========================================================
  // setup the mpc
  N_hor = 45;
  m_mpc = new ltv_mpc_filter(0.05, N_hor, m_cfg.robot.max_vel_x+0.2, m_cfg.robot.max_vel_theta + 0.2,
                         m_cfg.robot.acc_lim_x * m_cfg.robot.acc_filter_mutiplier, m_cfg.robot.acc_lim_theta * m_cfg.robot.acc_filter_mutiplier);
  //--init the mpc controller---
  std::vector<double> Qd, Rd, Sd;
  m_nh.getParam("/Qd",Qd);
  m_nh.getParam("/Rd",Rd);
  m_nh.getParam("/Sd",Sd);

  if (Qd.size() != 3 || Rd.size()!=2 || Sd.size() != 2)
  {
    std::cout<<"Cost matrix dimension is wrong!"<<std::endl;
    exit(-1);
  }

  Eigen::Matrix<double,3,3> Q;
  Q<<Qd[0], 0,     0,
     0,     Qd[1], 0,
     0,     0,     Qd[2];

  Eigen::Matrix<double,2,2> R;
  R<<Rd[0], 0,
     0,     Rd[1];

  Eigen::Matrix<double,2,2> S;
  S<<Sd[0], 0,
      0,    Sd[1];

  std::cout<<Q<<std::endl;
  std::cout<<"----------"<<std::endl;
  std::cout<<R<<std::endl;
  std::cout<<"----------"<<std::endl;
  std::cout<<S<<std::endl;
  std::cout<<"----------"<<std::endl;

  m_mpc->set_cost(Q,R,S);
}

IntLocalPlanner::~IntLocalPlanner()
{
  m_pso_planner->release();
  delete m_pso_planner;

  delete m_mpc;
}

void IntLocalPlanner::nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg)
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
  m_pso_planner->m_eva.m_nf1_map = *m_nf1_map;
  m_pso_planner->m_eva.m_nf1_received = true;

  // setup the drive type
  if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_FORWARD)
  {
    m_pso_planner->m_eva.m_pure_turning = false;
    m_pso_planner->m_eva.is_forward = true;
    m_teb_planner->m_is_forward = true;
  }
  else if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_BACKWARD)
  {
    m_pso_planner->m_eva.m_pure_turning = false;
    m_pso_planner->m_eva.is_forward = false;
    m_teb_planner->m_is_forward = false;
  }
  else if (msg->drive_type == cpc_aux_mapping::nf1_task::TYPE_ROTATE)
  {
    m_pso_planner->m_eva.m_pure_turning = true;
    m_pso_planner->m_eva.is_forward = true;
    m_teb_planner->m_is_forward = true;
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

void IntLocalPlanner::plan_call_back(const ros::TimerEvent&)
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
void IntLocalPlanner::do_start()
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
void IntLocalPlanner::do_normal()
{
  cycle_init();
  std::cout<<"NORMAL"<<std::endl;

  //Planning
  m_task_is_new = false;

  UGV::UGVModel::State ini_state = m_pso_planner->m_model.get_ini_state();

  auto start_time = std::chrono::steady_clock::now();
  if (!do_normal_teb())
  {
    std::cout<<"!!!emergent mode!!!"<<std::endl;
    do_normal_pso();
  }

  auto end_time = std::chrono::steady_clock::now();
      std::cout << "Local planning time: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                << " ms"<< std::endl;

  //Goto: Stuck
  if(is_stuck(m_traj,m_carrot) || is_stuck_lowpass(m_pso_planner->m_model.get_ini_state(),m_carrot))
  {
    m_teb_planner->clearPlanner();
    m_status = UGV::STUCK;
    m_stuck_start_cycle = m_plan_cycle;
  }

  //Goto: Pos_reached
  if(is_pos_reached(ini_state,m_goal.s,m_goal.reaching_radius))
  {
      float2 carrot_diff = ini_state.p - m_carrot.p;
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
bool IntLocalPlanner::do_normal_teb()
{
  UGV::UGVModel::State ini_state = m_pso_planner->m_model.get_ini_state();
  teb::PoseSE2 robot_pose(ini_state.p.x, ini_state.p.y, ini_state.theta);
  teb::PoseSE2 robot_goal(m_carrot.p.x, m_carrot.p.y, m_carrot.theta);
  geometry_msgs::Twist robot_vel;
  robot_vel.linear.x = ini_state.v;
  robot_vel.angular.z = ini_state.w;

  if (!m_teb_planner->get_edt_map())
    m_teb_planner->set_edt_map(m_edt_map);

  std::vector<double3> init = get_init_guess();
  //std::cout<<init.size()<<std::endl;
  m_teb_planner->set_init_plan(init);

  int checking_hor = m_use_simple_filter ? m_cfg.trajectory.feasibility_check_no_poses : N_hor;
  bool success = m_teb_planner->plan(robot_pose, robot_goal, checking_hor, &robot_vel, m_cfg.goal_tolerance.free_goal_vel);

  std_msgs::Int32 drive_dir;
  if (m_teb_planner->m_is_forward)
    drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_FORWARD;
  else
    drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_BACKWARD;

  m_drive_dir_pub.publish(drive_dir);

  if (!success)
  {
    m_teb_planner->clearPlanner();
    return false;
  }

  bool feasible = m_teb_planner->isTrajectoryFeasible();
  if(!feasible)
  {
    m_teb_planner->clearPlanner();
    return false;
  }

  std::cout<<"size: "<<m_teb_planner->bestTeb()->teb().sizePoses()<<std::endl;
  std::vector<teb::Reference> ref;

  if (m_teb_planner->bestTeb() && m_teb_planner->bestTeb()->get_reference(4,0.05, ref))
  {
    return smooth_reference(ini_state, ref, m_traj, m_use_simple_filter);
  }
  else
  {
    m_teb_planner->clearPlanner();
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
    return false;
  }
}

bool IntLocalPlanner::smooth_reference(const UGV::UGVModel::State &ini_state, const std::vector<teb::Reference> &raw_ref,
                                       std::vector<UGV::UGVModel::State> &final_ref, bool use_simple_filter)
{
  final_ref.clear();
  if (use_simple_filter)
  {
    float curr_v_r = ini_state.v;
    float curr_w_r = ini_state.w;
    for (teb::Reference r : raw_ref)
    {
      curr_v_r = acc_filter(curr_v_r, r.vx,    m_cfg.robot.acc_lim_x * m_cfg.robot.acc_filter_mutiplier,     0.05f);
      curr_w_r = acc_filter(curr_w_r, r.omega, m_cfg.robot.acc_lim_theta * m_cfg.robot.acc_filter_mutiplier, 0.05f);
      UGV::UGVModel::State s;
      s.p.x = r.pose.x();
      s.p.y = r.pose.y();
      s.v = curr_v_r;
      s.theta = r.pose.theta();
      s.w = curr_w_r;
      final_ref.push_back(s);
    }
    return true;
  }
  else
  {
    std::vector<double> x_i,y_i,th_i,v_i,w_i;
    for (teb::Reference r : raw_ref)
    {
      x_i.push_back(r.pose.x());
      y_i.push_back(r.pose.y());
      th_i.push_back(r.pose.theta());
      v_i.push_back(r.vx);
      w_i.push_back(r.omega);
    }

    m_mpc->set_reference_and_state(x_i, y_i, th_i, v_i, w_i,
                                   ini_state.p.x, ini_state.p.y, ini_state.theta,
                                   ini_state.v, ini_state.w);

    return m_mpc->solve(final_ref);
  }
}
//---
bool IntLocalPlanner::do_normal_pso()
{
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj, m_use_de);

  //Update the drive direction
  std_msgs::Int32 drive_dir;
  if(m_pso_planner->m_eva.m_using_auto_direction)
  {
    if (m_pso_planner->result.best_loc[0].z > 0)
      drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_FORWARD;
    else
      drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_BACKWARD;
  }
  else
  {
    if (m_pso_planner->m_eva.is_forward)
      drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_FORWARD;
    else
      drive_dir.data = cpc_aux_mapping::nf1_task::TYPE_BACKWARD;
  }

  m_drive_dir_pub.publish(drive_dir);

  if (m_pso_planner->result.collision)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }

  return true;
}
//=====================================
void IntLocalPlanner::do_stuck()
{
  cycle_init();
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
  if (m_plan_cycle - m_stuck_start_cycle >= 10)
  {
    m_status = UGV::NORMAL;
    m_pso_planner->m_eva.m_stuck = false;
  }
}
//=====================================
void IntLocalPlanner::do_emergent()
{
  cycle_init();
}
//=====================================
void IntLocalPlanner::do_braking()
{
  cycle_init();
  std::cout<<"BRAKING"<<std::endl;

  //Planning
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  //Goto: stuck - full stuck
  if (m_plan_cycle - m_braking_start_cycle >= 10)
  {
     m_status = UGV::STUCK;
     m_stuck_start_cycle = m_plan_cycle;
  }

}
//=====================================
void IntLocalPlanner::do_pos_reached()
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
void IntLocalPlanner::do_fully_reached()
{
  cycle_init();
  std::cout<<"FULLY_REACHED"<<std::endl;

  // Planing
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  //Go to the dropoff state
  m_status = UGV::DROPOFF;
}
//=====================================
void IntLocalPlanner::do_dropoff()
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
void IntLocalPlanner::cycle_init()
{
  if (cycle_initialized)
    return;

  float2 diff = m_goal.s.p - m_carrot.p;
  if (sqrtf(dot(diff,diff)) <= 2*m_edt_map->m_grid_step)
    m_pso_planner->m_eva.m_accurate_reaching = true;
  else
    m_pso_planner->m_eva.m_accurate_reaching = false;

  cycle_initialized = true;
  bool is_heading_ref;
  float psi = select_mes_ref_heading(is_heading_ref,get_heading(m_slam_odo), m_ref_theta, m_tht_err_reset_ctt, 0.25f);

  UGV::UGVModel::State s = predict_state(m_slam_odo,psi,m_ref_start_idx,is_heading_ref);

  s.v = select_mes_ref(m_raw_odo.twist.twist.linear.x, m_ref_v, m_v_err_reset_ctt);
  s.w = select_mes_ref(m_raw_odo.twist.twist.angular.z, m_ref_w, m_w_err_reset_ctt);

  m_pso_planner->m_model.set_ini_state(s);
  full_stop_trajectory(m_traj,s);

  //Publish the ref status string for loging
  std_msgs::String msg;
  std::stringstream ss;
  ss << "STT: " << m_status<<", CST:"<< m_pso_planner->result.best_cost<<", COL:"<<m_pso_planner->result.collision<<std::endl;
  msg.data = ss.str();
  m_status_pub.publish(msg);
}

//---
std::vector<double3> IntLocalPlanner::get_init_guess()
{
  std::vector<double3> pre_guess, guess;
  // the initial pose
  UGV::UGVModel::State ini_state = m_pso_planner->m_model.get_ini_state();
  pre_guess.push_back(make_double3(ini_state.p.x,
                                ini_state.p.y,
                                ini_state.theta));
  CUDA_GEO::pos p;
  p.x = ini_state.p.x;
  p.y = ini_state.p.y;
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

bool IntLocalPlanner::get_smallest_child(const CUDA_GEO::coord &c, CUDA_GEO::coord& bc)
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

