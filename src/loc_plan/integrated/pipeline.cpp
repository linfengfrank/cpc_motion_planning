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
    m_state->on_enter(); // Execute the state on enter
    m_state->prepare_props_for_checking(); // Set all proposition's checked flag as false
    m_state->check_props(); // Check all propositions
    m_state->exam_props_all_checked(); // Exam whether all propositions have been checked
    m_state->on_exit(); // Execute the state on exit
    m_state = &(m_state->toggle()); // Jump to the next state
  }
  finish_cycle(); // Finishing functions
}





