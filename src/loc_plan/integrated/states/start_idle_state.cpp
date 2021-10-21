#include "loc_plan/integrated/states/start_idle_state.h"
#include "loc_plan/integrated/local_planner_pipeline.h"
#include "loc_plan/integrated/states/normal_teb_state.h"

StartIdleState::StartIdleState()
{
  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(READY_TO_GO, &StartIdleState::check_ready_to_go);
}

void StartIdleState::on_enter()
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Start_Idile "<< m_p->get_cycle()<<std::endl;
#endif
  // Update the norminal yaw state
  if (m_p->m_bb.m_slam_odo_received)
    m_p->m_ref_theta = m_p->get_heading(m_p->m_bb.m_slam_odo);
}

void StartIdleState::on_exit()
{

}

void StartIdleState::attach_to_pipe(Pipeline *p)
{
  m_p = static_cast<LocalPlannerPipeline*>(p);
}

State& StartIdleState::toggle()
{
  if(is_true(READY_TO_GO))
  {
    return NormalTebState::getInstance();
  }
  else
  {
    return StartIdleState::getInstance();
  }
}

State& StartIdleState::getInstance()
{
  static StartIdleState singleton;
  return singleton;
}

bool StartIdleState::check_ready_to_go()
{
  if (m_p->m_bb.m_slam_odo_received &&
      m_p->m_bb.m_raw_odo_received &&
      m_p->m_bb.m_map_received &&
      m_p->m_bb.m_goal_received)
    return true;
  else
    return false;
}
