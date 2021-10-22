#include "loc_plan/integrated/pipeline.h"

Pipeline::Pipeline()
{
  m_state = nullptr;
  m_cycle = 0;
}

Pipeline::~Pipeline()
{

}

void Pipeline::execute(const ros::TimerEvent&)
{
  m_cycle_start_time = ros::Time::now();
  m_cycle++; // Increase the execution cycle
  m_token = 1; // Always give exactly 1 token in a new execution cycle
  prepare_cycle(); // Preparation functions
  while (m_token > 0)
  {
    m_token--; // Decrease 1 token when task loop is run once
    if (m_state_switch_detected)
    {
      m_state->on_activation();
      m_state_switch_detected = false;
    }
    m_state->on_execute(); // Execute the state on enter
    m_state->prepare_props_for_checking(); // Set all proposition's checked flag as false
    m_state->check_props(); // Check all propositions
    m_state->exam_props_all_checked(); // Exam whether all propositions have been checked
    m_state->on_finish(); // Execute the state on exit
    State * next_state = &(m_state->toggle());
    if (m_state != next_state)
    {
      m_state->on_deactivation();
      m_state = next_state;
      m_state_switch_detected = true;
    }
  }
  finish_cycle(); // Finishing functions
}





