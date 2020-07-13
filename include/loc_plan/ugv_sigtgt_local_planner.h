#ifndef UGV_MOTION_PLANNER_H
#define UGV_MOTION_PLANNER_H
#include <loc_plan/ugv_base_local_planner.h>

#define SIMPLE_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::SingleTargetEvaluator,UGV::UGVSwarm<2>
class UGVSigTgtMotionPlanner : public UGVLocalMotionPlanner
{
public:
  UGVSigTgtMotionPlanner();
  ~UGVSigTgtMotionPlanner();

protected:
  virtual void do_start();
  virtual void do_normal();
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
  bool m_goal_received;
  PSO::Planner<SIMPLE_UGV> *m_pso_planner;
  UGV::SingleTargetEvaluator::Target m_goal;
  float m_ref_v, m_ref_w;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;
};

#endif // UGV_MOTION_PLANNER_H
