#include "loc_plan/uav_astar_motion_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_srvs/Empty.h>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

UAVAstarMotionPlanner::UAVAstarMotionPlanner():
  UAVLocalMotionPlanner(),
  m_goal_received(false)
{
  m_goal_sub = m_nh.subscribe("/mid_layer/goal",1,&UAVAstarMotionPlanner::goal_call_back, this);

  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UAVAstarMotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<SIMPLE_UAV>(150,30,1);
  m_pso_planner->initialize();

  //Initialize the control message
  m_ref_msg.rows = 12;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;
}

UAVAstarMotionPlanner::~UAVAstarMotionPlanner()
{
  m_pso_planner->release();
  delete m_pso_planner;
}

void UAVAstarMotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  run_state();
  if (m_fly_status <= UAV::AT_GROUND)
    return;

  UAV::UAVModel::State s = m_curr_ref;

  m_head_sov.cal_from_pnt(m_goal.s.p,m_curr_ref);

    auto start = std::chrono::steady_clock::now();
  std::vector<UAV::UAVModel::State> traj;
  calculate_trajectory<SIMPLE_UAV>(m_pso_planner,traj,m_curr_ref,m_head_sov.get_yaw());





  auto end = std::chrono::steady_clock::now();
  std::cout << "Consumed: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost<<std::endl;

  std::vector<JLT::State> yaw_traj = m_head_sov.generate_yaw_traj();
  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;
  float t = 0.0f;
  int i = 0;
  for (UAV::UAVModel::State traj_s : traj)
  {
    t += PSO::PSO_CTRL_DT;
    JLT::State yaw_state = yaw_traj[i++];

    ref_counter++;
    add_to_ref_msg(m_ref_msg,ref_counter,traj_s,yaw_state);

    if (ref_counter == next_ref_start_idx)
    {
      m_curr_ref = traj_s;
      m_head_sov.set_yaw_state(yaw_state);
    }

    cols++;
  }

  m_ref_start_idx = next_ref_start_idx;
  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();

  m_plan_cycle++;
#ifdef SHOWPC
  plot_trajectory(traj);
#endif
}

void UAVAstarMotionPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_goal_received = true;
  m_goal.s.p.x = msg->pose.position.x;
  m_goal.s.p.y = msg->pose.position.y;
  m_goal.s.p.z = msg->pose.position.z;
  m_pso_planner->m_eva.setTarget(m_goal);

  UAV::UAVModel::State s = m_curr_ref;
  float3 diff = m_goal.s.p - s.p;
  diff.z = 0;
  float dist = sqrtf(dot(diff,diff));
  if (dist > 0.5f)
  {
      float theta = atan2f(diff.y,diff.x);
      m_pso_planner->m_ctrl_dev.m_theta = theta;
  }
}

void UAVAstarMotionPlanner::do_at_ground()
{
  if (m_pose_received && m_received_map && m_goal_received)
  {
    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("engage", eReq, eRes);
    m_fly_status = UAV::TAKING_OFF;
  }
}
void UAVAstarMotionPlanner::do_taking_off()
{
  if (m_curr_ref.p.z >= 1.8f && fabsf(m_curr_ref.v.z)<0.3f)
  {
    m_goal.oa = true;
    m_fly_status = UAV::IN_AIR;
  }
}
void UAVAstarMotionPlanner::do_in_air()
{
  if (0)
  {
    m_fly_status = UAV::EMERGENT;
  }
  else if (0)
  {
    m_fly_status = UAV::BRAKING;
  }
}
