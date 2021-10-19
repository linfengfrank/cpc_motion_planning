#include "loc_plan/integrated/pipeline.h"
#include "loc_plan/integrated/start_idle_state.h"
#include "tf/tf.h"

Pipeline::Pipeline()
{
  // Initialization
  m_state = &StartIdleState::getInstance();
  m_cycle = 0;

  //Start the timer
  m_planning_timer = m_nh.createTimer(ros::Duration(0.2), &Pipeline::execute, this);
}

void Pipeline::execute(const ros::TimerEvent&)
{
  m_cycle++; // Increase the execution cycle
  m_token = 1; // Always give exactly 1 token in a new execution cycle
  while (m_token > 0)
  {
    m_token--; // Decrease 1 token when task loop is run once
    m_state->on_enter(this); // Execute the state on enter
    m_state->prepare_props_for_checking(); // Set all proposition's checked flag as false
    m_state->check_props(this); // Check all propositions
    m_state->exam_props_all_checked(); // Exam whether all propositions have been checked
    m_state->on_exit(this); // Execute the state on exit
    m_state = &(m_state->toggle(this)); // Jump to the next state
  }
}


//---------------------------------------------------------
// Helper functions:
//---------------------------------------------------------
float Pipeline::get_heading(const nav_msgs::Odometry &odom)
{
  double phi,theta,psi;
  tf::Quaternion q(odom.pose.pose.orientation.x,
                   odom.pose.pose.orientation.y,
                   odom.pose.pose.orientation.z,
                   odom.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);
  return psi;
}
