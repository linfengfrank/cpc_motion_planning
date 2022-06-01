#ifndef UAV_NF1_MOTION_PLANNER_H
#define UAV_NF1_MOTION_PLANNER_H

#include <loc_plan/uav_base_local_planner.h>
#define SIMPLE_UAV_NF1 UAV::UAVModel,UAV::UAVDPControl,UAV::NF1Evaluator,UAV::UAVSwarm<1>
#define EMERGENT_UAV_NF1 UAV::UAVModel,UAV::UAVJLTControl,UAV::NF1Evaluator,UAV::UAVSwarm<1>

class UAVNF1MotionPlanner : public UAVLocalMotionPlanner
{
public:
  UAVNF1MotionPlanner();
  ~UAVNF1MotionPlanner();

protected:
  virtual void do_at_ground();
  virtual void do_taking_off();
  virtual void do_in_air();
  virtual void do_stuck();
  virtual void do_emergent();
  virtual void do_braking();

private:
  void plan_call_back(const ros::TimerEvent&);
  void nf1_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void cycle_init();
  void set_init_state(const UAV::UAVModel::State& trans, const JLT::State &yaw);

private:
  ros::Subscriber m_nf1_sub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  NF1Map *m_nf1_map;

  PSO::Planner<SIMPLE_UAV_NF1> *m_pso_planner;
  PSO::Planner<EMERGENT_UAV_NF1> *m_emergent_planner;
  UAVHeadingSolver m_head_sov;
  UAV::UAVRepulsiveField m_rep_filed;

  UAV::UAVModel::State m_curr_ref;
  JLT::State m_curr_yaw_ref;
  cpc_motion_planning::ref_data m_ref_msg;
  std::vector<UAV::UAVModel::State> m_traj;
  std::vector<JLT::State> m_yaw_traj;

  int m_start_braking_cycle;
  int m_start_stuck_cycle;
  int m_plan_cycle;
  int m_ref_start_idx;
  bool m_planning_started;
  bool m_goal_received;
  float m_take_off_height;
  float m_leap_height;
};

#endif // UAV_NF1_MOTION_PLANNER_H