#ifndef UGV_REFTRAJ_LOCAL_PLANNER_H
#define UGV_REFTRAJ_LOCAL_PLANNER_H

#include <loc_plan/ugv_base_local_planner.h>
#include <visualization_msgs/Marker.h>

#define REF_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::RefTrajEvaluator,UGV::UGVSwarm<2>

class UGVRefTrajMotionPlanner : public UGVLocalMotionPlanner
{
public:
  UGVRefTrajMotionPlanner();
  ~UGVRefTrajMotionPlanner();

protected:
  virtual void do_start();
  virtual void do_normal();
  virtual void do_stuck();
  virtual void do_emergent();
  virtual void do_braking();

private:
  void plan_call_back(const ros::TimerEvent&);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void cycle_init();
  void load_ref_lines();
  void calculate_ref_traj(float2 vehicle_pos);
  void linecirc_inter_dist(const float3 &seg_a, const float3 &seg_b, const float3 &circ_pos, float3 &closest, float &dist_v_len) const;
  float3 calculate_unit_vector(const float3 &seg_a, const float3 &seg_b);
  float calculate_length(const float3 &seg_a, const float3 &seg_b);

private:
  ros::Subscriber m_goal_sub;
  ros::Publisher m_vis_pub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  bool m_goal_received;
  PSO::Planner<REF_UGV> *m_pso_planner;
  UGV::RefTrajEvaluator::Ref m_ref_traj;
  float m_ref_v, m_ref_w;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;
  std::vector<UGV::UGVModel::State> m_traj;
  bool cycle_initialized;
  int m_braking_start_cycle;
  std::vector<float3> m_wps;
};

#endif // UGV_REFTRAJ_LOCAL_PLANNER_H
