#include "loc_plan/integrated/local_planner_pipeline.h"
#include "loc_plan/integrated/states/normal_teb_state.h"
#include "loc_plan/integrated/states/reach_state.h"

ReachState::ReachState()
{
  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(NEW_TARGET, &ReachState::check_new_target);
}

void ReachState::on_execute()
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Reach "<< m_p->get_cycle()<<std::endl;
#endif

  //Planning
 m_p->full_stop_trajectory(m_p->m_bb.m_ref_traj,m_p->m_bb.m_init_state);
}

void ReachState::on_finish()
{

}

void ReachState::attach_to_pipe(Pipeline *p)
{
  m_p = static_cast<LocalPlannerPipeline*>(p);
}

State& ReachState::toggle()
{
  if(is_true(NEW_TARGET))
  {
    return NormalTebState::getInstance();
  }
  else
  {
    return ReachState::getInstance();
  }
}

State& ReachState::getInstance()
{
  static ReachState singleton;
  return singleton;
}

bool ReachState::check_new_target()
{
  if (m_p->m_bb.m_task_is_new)
    return true;
  else
    return false;
}

void ReachState::on_activation()
{
  m_p->m_bb.m_task_is_new = false;
}
