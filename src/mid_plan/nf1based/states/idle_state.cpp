#include "mid_plan/nf1based/states/idle_state.h"
#include "mid_plan/nf1based/mid_planner_pipeline.h"

IdleState::IdleState()
{
  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(READY_TO_GO, &IdleState::check_ready_to_go);
}

void IdleState::on_execute()
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Idile "<< m_p->get_cycle()<<std::endl;
#endif
}

void IdleState::on_finish()
{

}

void IdleState::attach_to_pipe(Pipeline *p)
{
  m_p = static_cast<MidPlannerPipeline*>(p);
}

State& IdleState::toggle()
{
  if(is_true(READY_TO_GO))
  {
    return IdleState::getInstance();
  }
  else
  {
    return IdleState::getInstance();
  }
}

State& IdleState::getInstance()
{
  static IdleState singleton;
  return singleton;
}

bool IdleState::check_ready_to_go()
{
//  if (m_p->m_bb.m_slam_odo_received &&
//      m_p->m_bb.m_raw_odo_received &&
//      m_p->m_bb.m_map_received &&
//      m_p->m_bb.m_goal_received)
//    return true;
//  else
    return false;
}

