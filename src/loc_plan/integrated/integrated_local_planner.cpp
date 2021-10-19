#include "loc_plan/integrated/integrated_local_planner.h"
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
  float var_s, var_theta, step_dt, local_safety_radius, R_F, R_F_dot;
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
  m_nh.param<double>("/turning_efficiency",m_turning_efficiency,1.0);
  m_nh.param<bool>("/use_adrc",m_use_adrc,true);
  m_nh.param<bool>("/allow_update_max_speed",m_allow_update_max_speed,false);
  m_nh.param<float>("/R_F", R_F, 0.5f);
  m_nh.param<float>("/R_F_dot", R_F_dot, 0.5f);

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
  m_pso_planner->m_eva.m_footprint_offset = m_footprint_offset;
  m_pso_planner->m_eva.m_R_F = R_F;
  m_pso_planner->m_eva.m_R_F_dot = R_F_dot;

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

#ifdef SHOW_INIT_PLAN
  m_init_plan_pub = m_nh.advertise<PointCloud>("init_plan",1);
  m_init_plan_cld = PointCloud::Ptr(new PointCloud);
  m_init_plan_cld->header.frame_id = "/world";
#endif
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
  m_teb_planner->visualize();
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
void IntLocalPlanner::check_reach_and_stuck()
{
  UGV::UGVModel::State ini_state = m_pso_planner->m_model.get_ini_state();
  float2 carrot_diff = ini_state.p - m_carrot.p;
  //Check both goal and carrot are reached
  if(is_pos_reached(ini_state,m_goal.s,m_goal.reaching_radius) &&
     sqrtf(dot(carrot_diff,carrot_diff))<m_goal.reaching_radius)
  {
    m_status = UGV::POS_REACHED;
    std_msgs::Int32MultiArray reach_msg;
    reach_msg.data.push_back(m_goal.path_id);
    reach_msg.data.push_back(m_goal.act_id);
    m_tgt_reached_pub.publish(reach_msg);
  }
  else if(is_stuck(m_traj,m_carrot) ||
          is_stuck_lowpass(m_pso_planner->m_model.get_ini_state(),m_carrot))
  {
    // If not reached yet, check whether it is stucked
    m_teb_planner->clearPlanner();
    m_status = UGV::STUCK;
    m_stuck_start_cycle = m_plan_cycle;
  }
}

void IntLocalPlanner::update_max_speed(const std::vector<UGV::UGVModel::State> &traj)
{
  if (!m_allow_update_max_speed)
    return;

  float min_dist = traj_min_edt(traj);

  // If not using ADRC, also consider the simulated trajectory's minimum distance
  if (!m_use_adrc)
  {
    float sim_traj_min_dist = tracking_min_edt(m_slam_odo, m_traj, 2, m_turning_efficiency);

    if (sim_traj_min_dist < min_dist)
      min_dist = sim_traj_min_dist;
  }
  float new_max_speed = 0.5f*min_dist;
  new_max_speed = new_max_speed<0.2f ? 0.2f : new_max_speed;
  new_max_speed = new_max_speed>0.7f ? 0.7f : new_max_speed;

  std::cout<<"new_max_speed: "<<new_max_speed<<std::endl;
  m_cfg.robot.max_vel_x = new_max_speed;
  m_cfg.robot.max_vel_x_backwards = new_max_speed;

  m_pso_planner->m_swarm.set_var_s(2*new_max_speed);
}

void IntLocalPlanner::do_normal()
{
  cycle_init();
  std::cout<<"NORMAL"<<std::endl;

  //Planning
  m_task_is_new = false;

  // First try TEB planner
  if (do_normal_teb())
  {
    update_max_speed(m_traj);
    publish_drive_direction_teb();
    check_reach_and_stuck();
  }
  else if (do_normal_pso()) //If it does not work try PSO planner
  {
    std::cout<<"!!!PSO (emergent) mode!!!"<<std::endl;
    update_max_speed(m_traj);
    publish_drive_direction_pso();
    check_reach_and_stuck();
  }
  else // All planer fails, BREAK the vehicle
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }
}

bool IntLocalPlanner::do_normal_teb()
{
  // Setup the initial pose
  UGV::UGVModel::State ini_state = m_pso_planner->m_model.get_ini_state();
  teb::PoseSE2 robot_pose(ini_state.p.x, ini_state.p.y, ini_state.theta);

  // Setup the goal pose
  teb::PoseSE2 robot_goal(m_carrot.p.x, m_carrot.p.y, m_carrot.theta);

  // Setup the initial velocity
  geometry_msgs::Twist robot_vel;
  robot_vel.linear.x = ini_state.v;
  robot_vel.angular.z = ini_state.w;

  // Setup the EDT map
  if (!m_teb_planner->get_edt_map())
    m_teb_planner->set_edt_map(m_edt_map);

  // Get and set the initial guess of the trajectory
  std::vector<double2> init = get_init_path_guess();
  //std::cout<<init.size()<<std::endl;
  m_teb_planner->set_init_plan(init);

  // If we are using the ltv mpc to do the smoothing, the checking horizon shall be the MPC's horizon
  int checking_hor = m_use_simple_filter ? m_cfg.trajectory.feasibility_check_no_poses : N_hor;

  // Do the planning
  bool success = m_teb_planner->plan(robot_pose, robot_goal, checking_hor, &robot_vel, m_teb_planner->m_is_forward);

  // Check the planning results
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

  // Try read out the reference from the planning result (a list of poses)
  std::vector<teb::Reference> ref;
  if (!(m_teb_planner->bestTeb() && m_teb_planner->bestTeb()->get_reference(4,0.05, ref)))
  {
    // If cannot get the ref, clear the planner as its answer is not correct
    m_teb_planner->clearPlanner();
    return false;
  }

  // If cannot smooth the reference, return false
  if (!smooth_reference(ini_state, ref, m_traj, m_use_simple_filter))
  {
    return false;
  }

  // If not in ADRC mode (tracking mode), the simulated trajectory might collide with obstacle
  if (!is_tracking_safe(m_slam_odo, m_traj))
  {
    return false;
  }

  return true;
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

  // Check whether the planned trajectory is successful (has collision or not)
  if (m_pso_planner->result.collision)
  {
    return false;
  }

  // If not in ADRC mode (tracking mode), the simulated trajectory might collide with obstacle
  if (!is_tracking_safe(m_slam_odo, m_traj))
  {
    return false;
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

  if (m_pso_planner->result.collision ||
      !is_tracking_safe(m_slam_odo, m_traj))
  {
    //Goto: Braking
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }
  else if (m_plan_cycle - m_stuck_start_cycle >= 10)
  {
    //Goto: Normal
    m_status = UGV::NORMAL;
    m_pso_planner->m_eva.m_stuck = false;
  }
  // else stays in this state
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
  float psi;

  if(m_use_adrc)
  {
    psi = get_heading(m_slam_odo);
    is_heading_ref = false;
  }
  else
  {
    psi = select_mes_ref_heading(is_heading_ref,get_heading(m_slam_odo), m_ref_theta, m_tht_err_reset_ctt, 0.25f);
  }

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
std::vector<double2> IntLocalPlanner::get_init_path_guess()
{
  std::vector<double2> guess;
  UGV::UGVModel::State ini_state = m_pso_planner->m_model.get_ini_state();

  CUDA_GEO::pos p;
  p.x = ini_state.p.x;
  p.y = ini_state.p.y;
  guess.push_back(make_double2(p.x, p.y));
  CUDA_GEO::coord c = m_nf1_map->pos2coord(p);
  CUDA_GEO::coord bc;
  // First calculate the positions
  while(1)
  {
    if(get_smallest_child(c,bc))
    {
      c = bc;
      p = m_nf1_map->coord2pos(c);
      guess.push_back(make_double2(p.x, p.y));
    }
    else
    {
      break;
    }
  }

#ifdef SHOW_INIT_PLAN
  for (double2 pnt: guess)
  {
    pcl::PointXYZ clrP;
    clrP.x = pnt.x;
    clrP.y = pnt.y;
    clrP.z = 0;
    m_init_plan_cld->points.push_back(clrP);
  }
  m_init_plan_pub.publish(m_init_plan_cld);
  m_init_plan_cld->clear();
#endif

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

