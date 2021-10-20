#include "loc_plan/integrated/start_idle_state.h"
#include "loc_plan/integrated/pipeline.h"
#include "loc_plan/integrated/normal_teb_state.h"

StartIdleState::StartIdleState()
{
  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(READY_TO_GO, &StartIdleState::check_ready_to_go);
}

void StartIdleState::on_enter(Pipeline *p)
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Start_Idile "<< p->get_cycle()<<std::endl;
#endif
  // Update the norminal yaw state
  if (p->m_bb.m_slam_odo_received)
    p->m_ref_theta = p->get_heading(p->m_bb.m_slam_odo);
}

void StartIdleState::on_exit(Pipeline *p)
{

}

State& StartIdleState::toggle(Pipeline* p)
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

bool StartIdleState::check_ready_to_go(Pipeline *p)
{
  if (p->m_bb.m_slam_odo_received &&
      p->m_bb.m_raw_odo_received &&
      p->m_bb.m_map_received &&
      p->m_bb.m_goal_received)
    return true;
  else
    return false;
}
