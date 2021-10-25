#include "loc_plan/integrated/states/brake_state.h"
#include "loc_plan/integrated/local_planner_pipeline.h"
#include "loc_plan/integrated/states/normal_teb_state.h"
#include "loc_plan/integrated/states/stuck_state.h"

BrakeState::BrakeState()
{
  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(TIME_UP, &BrakeState::check_time_up);
}

void BrakeState::on_execute()
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Brake "<< m_p->get_cycle()<<std::endl;
#endif

  //Planning
 m_p->full_stop_trajectory(m_p->m_bb.m_ref_traj,m_p->m_bb.m_init_state);
}

void BrakeState::on_finish()
{

}

void BrakeState::attach_to_pipe(Pipeline *p)
{
  m_p = static_cast<LocalPlannerPipeline*>(p);
}

State& BrakeState::toggle()
{
  if(is_true(TIME_UP))
  {
    return StuckState::getInstance();
  }
  else
  {
    return BrakeState::getInstance();
  }
}

State& BrakeState::getInstance()
{
  static BrakeState singleton;
  return singleton;
}

bool BrakeState::check_time_up()
{
  if (m_p->get_cycle() - m_brake_start_cycle > 10)
    return true;
  else
    return false;
}

void BrakeState::on_activation()
{
  m_brake_start_cycle = m_p->get_cycle();
}
