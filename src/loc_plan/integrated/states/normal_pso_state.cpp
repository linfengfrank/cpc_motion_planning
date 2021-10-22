#include "loc_plan/integrated/states/normal_pso_state.h"
#include "loc_plan/integrated/local_planner_pipeline.h"

NormalPsoState::NormalPsoState()
{
  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(STUCK, &NormalPsoState::check_stuck);
  ADD_PROPOSITION(REACH, &NormalPsoState::check_reach);
  ADD_PROPOSITION(SUCCESS, &NormalPsoState::check_success);

  // Read in the parameters
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
  m_nh.param<float>("/R_F", R_F, 0.5f);
  m_nh.param<float>("/R_F_dot", R_F_dot, 0.5f);

  // Initialize the PSO planner
  m_pso_planner = new PSO::Planner<SIMPLE_UGV>(m_swarm_size,m_batch_num,m_episode_num);

  // Initialize the swarm
  m_pso_planner->m_swarm.set_step_dt(step_num, step_dt);
  m_pso_planner->m_swarm.set_var(make_float3(var_s,var_theta,1.0f));
  m_pso_planner->m_eva.m_using_auto_direction = use_auto_direction;
  m_pso_planner->m_eva.m_safety_radius = local_safety_radius;
  m_pso_planner->m_eva.m_footprint_offset = m_footprint_offset;
  m_pso_planner->m_eva.m_R_F = R_F;
  m_pso_planner->m_eva.m_R_F_dot = R_F_dot;

  m_pso_planner->m_file_location = dp_file_location;
  m_pso_planner->initialize();
}

void NormalPsoState::attach_to_pipe(Pipeline *p)
{
  m_p = static_cast<LocalPlannerPipeline*>(p);
}

void NormalPsoState::on_enter()
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Normal_PSO "<< m_p->get_cycle()<<std::endl;
#endif

  // Set up the initial state
  m_pso_planner->m_model.set_ini_state(m_p->m_bb.m_init_state);

  // Set up the NF1 map
  m_pso_planner->m_eva.m_nf1_map = *(m_p->m_bb.m_nf1_map);
  m_pso_planner->m_eva.m_nf1_received = true;

  // Setup the drive type
  set_driving_dir();

  // Setup the goal
  m_pso_planner->m_eva.setTarget(m_p->m_bb.m_goal);

  // Do the actual PSO planning
  m_plan_success = true;
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_p->m_bb.m_edt_map, m_pso_traj, m_use_de);

  // Check whether the planned trajectory is successful (has collision or not)
  if (m_pso_planner->result.collision)
  {
    m_plan_success = false;
  }

  // If not in ADRC mode (tracking mode), the simulated trajectory might collide with obstacle
  if (!m_p->is_tracking_safe(m_p->m_bb.m_slam_odo, m_pso_traj))
  {
    m_plan_success = false;
  }
}

void NormalPsoState::on_exit()
{
  if (is_true(SUCCESS))
  {
    m_p->m_bb.m_ref_traj = m_pso_traj;
    set_exec_drive_direction();
  }
  else
  {
    m_p->full_stop_trajectory(m_p->m_bb.m_ref_traj,m_p->m_bb.m_init_state);
  }
}

State& NormalPsoState::toggle()
{
  return NormalPsoState::getInstance();
}

void NormalPsoState::check_props()
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

State & NormalPsoState::getInstance()
{
  static NormalPsoState singleton;
  return singleton;
}

bool NormalPsoState::check_stuck()
{
  if(m_p->is_stuck(m_pso_traj, m_p->m_bb.m_carrot) ||
     m_p->is_stuck_lowpass(m_p->m_bb.m_init_state,m_p->m_bb.m_carrot))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool NormalPsoState::check_reach()
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

bool NormalPsoState::check_success()
{
  if (m_plan_success)
    return true;
  else
    return false;
}

void NormalPsoState::set_driving_dir()
{
  // setup the drive type
  if (m_p->m_bb.m_task_drive_dir == cpc_aux_mapping::nf1_task::TYPE_FORWARD)
  {
    m_pso_planner->m_eva.m_pure_turning = false;
    m_pso_planner->m_eva.is_forward = true;
  }
  else if (m_p->m_bb.m_task_drive_dir == cpc_aux_mapping::nf1_task::TYPE_BACKWARD)
  {
    m_pso_planner->m_eva.m_pure_turning = false;
    m_pso_planner->m_eva.is_forward = false;
  }
  else if (m_p->m_bb.m_task_drive_dir == cpc_aux_mapping::nf1_task::TYPE_ROTATE)
  {
    m_pso_planner->m_eva.m_pure_turning = true;
    m_pso_planner->m_eva.is_forward = true;
  }
}

void NormalPsoState::set_exec_drive_direction()
{
  // For the PSO planner, its driving direction can be changed in the planner
  // if m_using_auto_direction is true

  // If auto_direction is true, we take the driving direction of
  // the first segment of PSO's trajectory
  // m_pso_planner->result.best_loc[0].z > 0 means forward, < 0 means backward.
  // If the auto_direction is false, then just use the  is_forward data
  // stored inside the evaluator (shall be assigned from the NF1 map).
  if(m_pso_planner->m_eva.m_using_auto_direction)
    m_p->m_bb.m_exec_drive_dir = m_p->bool_to_drive_type(m_pso_planner->result.best_loc[0].z > 0);
  else
    m_p->m_bb.m_exec_drive_dir = m_p->bool_to_drive_type(m_pso_planner->m_eva.is_forward);
}



