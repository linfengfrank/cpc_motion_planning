#ifndef UGV_MOTION_PLANNER_H
#define UGV_MOTION_PLANNER_H
#include <loc_plan/ugv_base_local_planner.h>
#include <cpc_motion_planning/line_target.h>
#include <cpc_motion_planning/path.h>
#include <std_msgs/Int32.h>
#include <recover_plan/ugv_recover_planner.h>

#define SIMPLE_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::SingleTargetEvaluator,UGV::UGVSwarm<3>
class UGVSigTgtMotionPlanner : public UGVLocalMotionPlanner
{
  enum STUCK_SUB_MODE
  {
    RECOVER = 0,
    FULL_STUCK
  };

public:
  UGVSigTgtMotionPlanner();
  ~UGVSigTgtMotionPlanner();

protected:
  virtual void do_start();
  virtual void do_normal();
  virtual void do_stuck();
  virtual void do_emergent();
  virtual void do_braking();
  virtual void do_pos_reached();
  virtual void do_fully_reached();
  virtual void do_dropoff();
private:
  void do_recover();
  void do_full_stuck();
private:
  void plan_call_back(const ros::TimerEvent&);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void mid_goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void nf1_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void cycle_init();
  void line_target_call_back(const cpc_motion_planning::line_target::ConstPtr &msg);
  void hybrid_path_call_back(const cpc_motion_planning::path::ConstPtr &msg);

private:
  ros::Subscriber m_goal_sub;
  ros::Subscriber m_mid_goal_sub;
  ros::Subscriber m_nf1_sub;
  ros::Subscriber m_line_tgt_sub;
  ros::Subscriber m_hybrid_path_sub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  ros::Publisher m_status_pub;
  ros::Publisher m_tgt_reached_pub;
  ros::Publisher m_stuck_plan_request_pub;

  bool m_goal_received;
  PSO::Planner<SIMPLE_UGV> *m_pso_planner;
  UGV::SingleTargetEvaluator::Target m_goal;
  float2 m_mid_goal;
  bool m_mid_goal_received;
  float m_ref_v, m_ref_w, m_ref_theta;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt, m_tht_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;
  std::vector<UGV::UGVModel::State> m_traj;
  bool cycle_initialized;
  int m_braking_start_cycle;
  int m_plan_request_cycle;
  NF1MapDT *m_nf1_map;
  cpc_motion_planning::path m_stuck_recover_path;
  UGVRecMotionPlanner m_recover_planner;
  ros::ServiceClient m_collision_check_client;
  STUCK_SUB_MODE m_stuck_submode;
  int m_full_stuck_ctt;
};

#endif // UGV_MOTION_PLANNER_H
