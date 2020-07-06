#ifndef UAV_VEL_MOTION_PLANNER_H
#define UAV_VEL_MOTION_PLANNER_H
#include <loc_plan/uav_base_local_planner.h>

#define VEL_UAV UAV::UAVModel,UAV::UAVDPControl,UAV::SingleTargetEvaluator,UAV::UAVSwarm<2>
#define EMERGENT_UAV UAV::UAVModel,UAV::UAVJLTControl,UAV::SingleTargetEvaluator,UAV::UAVSwarm<1>

class UAVVelMotionPlanner : public UAVLocalMotionPlanner
{
public:
  UAVVelMotionPlanner();
  ~UAVVelMotionPlanner();

protected:
  virtual void do_at_ground();
  virtual void do_taking_off();
  virtual void do_in_air();
  virtual void do_stuck();
  virtual void do_emergent();
  virtual void do_braking();

private:
  void plan_call_back(const ros::TimerEvent&);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void cycle_init();

private:
  ros::Subscriber m_goal_sub;
  ros::Timer m_planning_timer;

  ros::Publisher m_ref_pub;
  ros::Publisher topology_paths_pub;

  bool m_goal_received;

  PSO::Planner<VEL_UAV> *m_pso_planner;
  PSO::Planner<EMERGENT_UAV> *m_emergent_planner;
  UAV::SingleTargetEvaluator::Target m_goal;
  UAV::UAVModel::State m_curr_ref;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_plan_cycle;
  int m_ref_start_idx;
  UAVHeadingSolver m_head_sov;
  std::vector<UAV::UAVModel::State> m_traj;
  std::vector<JLT::State> m_yaw_traj;
  int m_start_braking_cycle;
  UAV::UAVRepulsiveField m_rep_filed;
};
#endif // UAV_VEL_MOTION_PLANNER_H
