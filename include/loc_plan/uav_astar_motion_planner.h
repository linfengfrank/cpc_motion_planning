#ifndef UAV_ASTAR_MOTION_PLANNER_H
#define UAV_ASTAR_MOTION_PLANNER_H
#include <loc_plan/uav_local_motion_planner.h>

#define SIMPLE_UAV UAV::UAVModel,UAV::UAVDPControl,UAV::SingleTargetEvaluator,UAV::UAVSwarm<1>
#define CORRID_UAV UAV::UAVModel,UAV::UAVDPControl,UAV::CorridorEvaluator,UAV::UAVSwarm<1>
class UAVAstarMotionPlanner : public UAVLocalMotionPlanner
{
public:
  UAVAstarMotionPlanner();
  ~UAVAstarMotionPlanner();

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

private:
  ros::Subscriber m_goal_sub;
  ros::Timer m_planning_timer;

  ros::Publisher m_ref_pub;
  ros::Publisher topology_paths_pub;

  bool m_goal_received;

  PSO::Planner<SIMPLE_UAV> *m_pso_planner;

  UAV::SingleTargetEvaluator::Target m_goal;
  UAV::UAVModel::State m_curr_ref;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_plan_cycle;
  int m_ref_start_idx;
  UAVHeadingSolver m_head_sov;
};

#endif // UAV_ASTAR_MOTION_PLANNER_H
