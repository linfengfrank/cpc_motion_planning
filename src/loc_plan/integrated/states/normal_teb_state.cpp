#include "loc_plan/integrated/states/normal_teb_state.h"
#include "loc_plan/integrated/local_planner_pipeline.h"
#include "loc_plan/integrated/states/normal_pso_state.h"
#include "loc_plan/integrated/states/stuck_state.h"
#include "loc_plan/integrated/states/reach_state.h"
NormalTebState::NormalTebState()
{
  // Read in the parameters
  m_nh.param<bool>("/use_simple_filter",m_use_simple_filter,true);

  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(STUCK, &NormalTebState::check_stuck);
  ADD_PROPOSITION(REACH, &NormalTebState::check_reach);
  ADD_PROPOSITION(SUCCESS, &NormalTebState::check_success);

  // Init the TEB planner
  m_cfg.loadRosParamFromNodeHandle(m_nh);
  m_cfg.map_frame = "world";
  m_visualization = teb::TebVisualizationPtr(new teb::TebVisualization(m_nh, m_cfg));
  m_teb_planner = teb::HomotopyClassPlannerPtr(new teb::HomotopyClassPlanner(m_cfg, m_visualization));

  if (!m_use_simple_filter)
  {
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

}

void NormalTebState::attach_to_pipe(Pipeline *p)
{
  m_p = static_cast<LocalPlannerPipeline*>(p);
}

void NormalTebState::on_execute()
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Normal TEB "<< m_p->get_cycle()<<std::endl;
#endif

  // Set driving direction
  set_driving_dir();

  // Setup the initial pose
  UGV::UGVModel::State ini_state = m_p->m_bb.m_init_state;
  teb::PoseSE2 robot_pose(ini_state.p.x, ini_state.p.y, ini_state.theta);

  // Setup the goal pose
  teb::PoseSE2 robot_goal(m_p->m_bb.m_carrot.p.x, m_p->m_bb.m_carrot.p.y, m_p->m_bb.m_carrot.theta);

  // Setup the initial velocity
  geometry_msgs::Twist robot_vel;
  robot_vel.linear.x = ini_state.v;
  robot_vel.angular.z = ini_state.w;

  // Setup the EDT map
  if (!m_teb_planner->get_edt_map())
    m_teb_planner->set_edt_map(m_p->m_bb.m_edt_map);

  // Get and set the initial guess of the trajectory
  std::vector<double2> init = get_init_path_guess();
  //std::cout<<init.size()<<std::endl;
  m_teb_planner->set_init_plan(init);

  // If we are using the ltv mpc to do the smoothing, the checking horizon shall be the MPC's horizon
  int checking_hor = m_use_simple_filter ? m_cfg.trajectory.feasibility_check_no_poses : N_hor;

  // Do the planning
  m_plan_success = m_teb_planner->plan(robot_pose, robot_goal, checking_hor, &robot_vel, m_teb_planner->m_is_forward);

  // Check the planning results
  if (!m_plan_success)
  {
    m_teb_planner->clearPlanner();
    return;
  }

  m_plan_success = m_teb_planner->isTrajectoryFeasible();
  if(!m_plan_success)
  {
    m_teb_planner->clearPlanner();
    return;
  }

  // Try read out the reference from the planning result (a list of poses)
  std::vector<teb::Reference> ref;
  if (!(m_teb_planner->bestTeb() && m_teb_planner->bestTeb()->get_reference(4,0.05, ref)))
  {
    // If cannot get the ref, clear the planner as its answer is not correct
    m_teb_planner->clearPlanner();
    m_plan_success = false;
    return;
  }

  // If cannot smooth the reference, return false
  if (!smooth_reference(ini_state, ref, m_teb_traj))
  {
    m_plan_success = false;
    return;
  }

  // If not in ADRC mode (tracking mode), the simulated trajectory might collide with obstacle
  if (!m_p->is_tracking_safe(m_p->m_bb.m_slam_odo, m_teb_traj))
  {
    m_plan_success = false;
    return;
  }
}

void NormalTebState::on_finish()
{
  if (is_true(SUCCESS))
  {
    m_p->m_bb.m_ref_traj = m_teb_traj;
    set_exec_drive_direction();
  }
  else
  {
    m_p->full_stop_trajectory(m_p->m_bb.m_ref_traj,m_p->m_bb.m_init_state);
  }
}

State& NormalTebState::toggle()
{
  if (is_true(SUCCESS))
  {
    if(is_true(REACH))
    {
      return ReachState::getInstance();
    }
    else if (is_true(STUCK))
    {
      return StuckState::getInstance();
    }
    else
    {
      return NormalTebState::getInstance();
    }
  }
  else
  {
    m_p->add_token();
    return NormalPsoState::getInstance();
  }
//  return NormalTebState::getInstance();

}

State& NormalTebState::getInstance()
{
  static NormalTebState singleton;
  return singleton;
}

void NormalTebState::check_props()
{
  // First check wheter the planning is successful
  check_prop(SUCCESS);

  // Only check for reach and stuck when the plan is successful
  if (is_true(SUCCESS))
  {
    check_prop(REACH);

    // If it already reached the target, no need to check STUCK.
    if(!is_true(REACH))
      check_prop(STUCK);
    else
      set_prop(STUCK,false);
  }
  else
  {
    set_prop(REACH,false);
    set_prop(STUCK,false);
  }
}

bool NormalTebState::check_stuck()
{
  if(m_p->is_stuck(m_teb_traj, m_p->m_bb.m_carrot) ||
     m_p->is_stuck_lowpass(m_p->m_bb.m_init_state,m_p->m_bb.m_carrot))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool NormalTebState::check_reach()
{
  UGV::NF1Evaluator::Target goal = m_p->m_bb.m_goal;
  UGV::UGVModel::State ini_state = m_p->m_bb.m_init_state;
  float2 carrot_diff = ini_state.p - m_p->m_bb.m_carrot.p;
  //Check both goal and carrot are reached
  if(m_p->is_pos_reached(ini_state, goal.s, goal.reaching_radius) &&
     sqrtf(dot(carrot_diff,carrot_diff)) < goal.reaching_radius)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool NormalTebState::check_success()
{
  if (m_plan_success)
    return true;
  else
    return false;
}

std::vector<double2> NormalTebState::get_init_path_guess()
{
  std::vector<double2> guess;
  UGV::UGVModel::State ini_state = m_p->m_bb.m_init_state;

  CUDA_GEO::pos p;
  p.x = ini_state.p.x;
  p.y = ini_state.p.y;
  guess.push_back(make_double2(p.x, p.y));
  CUDA_GEO::coord c = m_p->m_bb.m_nf1_map->pos2coord(p);
  CUDA_GEO::coord bc;
  // First calculate the positions
  while(1)
  {
    if(get_smallest_child(m_p->m_bb.m_nf1_map,c,bc))
    {
      c = bc;
      p = m_p->m_bb.m_nf1_map->coord2pos(c);
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

bool NormalTebState::get_smallest_child(NF1MapDT *nf1_map, const CUDA_GEO::coord &c, CUDA_GEO::coord& bc)
{
  CUDA_GEO::coord g;
  bool found = false;
  float best_cost = nf1_map->getCostHst(c.x,c.y,0);
  for (int i=-1;i<=1;i++)
  {
    for(int j=-1;j<=1;j++)
    {
      if(i==0 && j==0)
        continue;

      g.x = c.x+i;
      g.y = c.y+j;
      float cost = nf1_map->getCostHst(g.x,g.y,0);
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

bool NormalTebState::smooth_reference(const UGV::UGVModel::State &ini_state, const std::vector<teb::Reference> &raw_ref,
                                       std::vector<UGV::UGVModel::State> &final_ref)
{
  final_ref.clear();
  if (m_use_simple_filter)
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

float NormalTebState::acc_filter(float curr_v, float tgt_v, const float acc_lim, const float dt)
{
  float achieved_v;
  float desired_acc = (tgt_v - curr_v)/dt;

  if(fabsf(desired_acc) < acc_lim)
  {
    achieved_v = tgt_v;
  }
  else
  {
    if (desired_acc > 0)
      achieved_v = curr_v + acc_lim*dt;
    else
      achieved_v = curr_v - acc_lim*dt;
  }
  return achieved_v;
}

void NormalTebState::set_driving_dir()
{
  // setup the drive type
  if (m_p->m_bb.m_task_drive_dir == cpc_aux_mapping::nf1_task::TYPE_FORWARD)
    m_teb_planner->m_is_forward = true;

  else if (m_p->m_bb.m_task_drive_dir == cpc_aux_mapping::nf1_task::TYPE_BACKWARD)
    m_teb_planner->m_is_forward = false;

  else if (m_p->m_bb.m_task_drive_dir == cpc_aux_mapping::nf1_task::TYPE_ROTATE)
    m_teb_planner->m_is_forward = true;
}

void NormalTebState::set_exec_drive_direction()
{
  // For the TEB planner, its driving direction is pre-determined
  m_p->m_bb.m_exec_drive_dir = m_p->bool_to_drive_type(m_teb_planner->m_is_forward);
}


