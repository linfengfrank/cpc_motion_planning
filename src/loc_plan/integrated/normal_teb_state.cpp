#include "loc_plan/integrated/normal_teb_state.h"
#include "loc_plan/integrated/pipeline.h"

NormalTebState::NormalTebState()
{
  // Resize the proposition container
  INIT_PROPOSITION_VECTOR();

  // Add in the propositions
  ADD_PROPOSITION(STUCK, &NormalTebState::check_stuck);
  ADD_PROPOSITION(REACH, &NormalTebState::check_reach);
}

void NormalTebState::on_enter(Pipeline *p)
{
#ifdef PRINT_STATE_DEBUG_INFO
  std::cout<<"Normal TEB "<< p->get_cycle()<<std::endl;
#endif
}

void NormalTebState::on_exit(Pipeline *p)
{

}

State& NormalTebState::toggle(Pipeline* pipe)
{
  if(is_true(REACH))
  {
    return NormalTebState::getInstance();
  }
  else if (is_true(STUCK))
  {
    return NormalTebState::getInstance();
  }
  else
  {
    return NormalTebState::getInstance();
  }
}

//void NormalTebState::check_proposition(Pipeline* pipe)
//{
//    for (Proposition &p : m_props)
//    {
//        p.evaulate(pipe);
//        break;
//    }
//}

State& NormalTebState::getInstance()
{
  static NormalTebState singleton;
  return singleton;
}

bool NormalTebState::check_stuck(Pipeline* pipe)
{
  return true;
}

bool NormalTebState::check_reach(Pipeline* pipe)
{
  return true;
}
