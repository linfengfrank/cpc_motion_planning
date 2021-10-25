#include "loc_plan/integrated/states/stuck_state.h"
#include "loc_plan/integrated/local_planner_pipeline.h"
#include "loc_plan/integrated/states/normal_teb_state.h"
#include "loc_plan/integrated/states/normal_pso_state.h"
#include "loc_plan/integrated/states/brake_state.h"
StuckState::StuckState()
{
  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(TIME_UP, &StuckState::check_time_up);
  ADD_PROPOSITION(SUCCESS, &StuckState::check_success);
}

void StuckState::on_execute()
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Stuck "<< m_p->get_cycle()<<std::endl;
#endif

  m_plan_success = m_pso->run_pso_planner(m_stuck_traj,true);

}

void StuckState::on_finish()
{
  if (is_true(SUCCESS))
  {
    m_p->m_bb.m_ref_traj = m_stuck_traj;
  }
  else
  {
    m_p->full_stop_trajectory(m_p->m_bb.m_ref_traj,m_p->m_bb.m_init_state);
  }
}

void StuckState::attach_to_pipe(Pipeline *p)
{
  m_p = static_cast<LocalPlannerPipeline*>(p);
}

State& StuckState::toggle()
{
  if(is_true(SUCCESS))
  {
    if(is_true(TIME_UP))
    {
      return NormalTebState::getInstance();
    }
    else
    {
      return StuckState::getInstance();
    }
  }
  else
  {
    m_p->add_token();
    return BrakeState::getInstance();
  }
}

State& StuckState::getInstance()
{
  static StuckState singleton;
  return singleton;
}

bool StuckState::check_time_up()
{
  if (m_p->get_cycle() - m_stuck_start_cycle > 10)
    return true;
  else
    return false;
}

void StuckState::on_activation()
{
  if (!m_pso)
  {
    m_pso = static_cast<NormalPsoState*>(&NormalPsoState::getInstance());
  }

  m_stuck_start_cycle = m_p->get_cycle();
}

bool StuckState::check_success()
{
  if (m_plan_success)
    return true;
  else
    return false;
}
