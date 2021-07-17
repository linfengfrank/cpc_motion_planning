#ifndef UGV_TEB_PLANNER_H
#define UGV_TEB_PLANNER_H
#include <loc_plan/ugv_base_local_planner.h>
#include <cpc_motion_planning/path.h>
#include <std_msgs/Int32.h>
#include <pure_pursuit/pure_pursuit_ctrl.h>
#include <cpc_aux_mapping/nf1_task.h>

#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>

#define SIMPLE_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::NF1Evaluator,UGV::UGVSwarm<8>
namespace teb_local_planner
{
class TEBLocalPlanner : public UGVLocalMotionPlanner
{
  enum STUCK_SUB_MODE
  {
    RECOVER = 0,
    FULL_STUCK
  };

public:
  TEBLocalPlanner();
  ~TEBLocalPlanner();

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
  void nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg);
  void cycle_init();
  std::vector<double2> get_init_path_guess();
  bool get_smallest_child(const CUDA_GEO::coord &c, CUDA_GEO::coord &bc);
  bool check_tgt_is_same(const UGV::NF1Evaluator::Target &t1, const UGV::NF1Evaluator::Target &t2)
  {
    if (t1.act_id == t2.act_id && t1.path_id == t2.path_id)
      return true;
    else
      return false;
  }

private:
  ros::Subscriber m_nf1_sub;
  ros::Subscriber m_hybrid_path_sub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  ros::Publisher m_status_pub;
  ros::Publisher m_tgt_reached_pub;
  ros::Publisher m_stuck_plan_request_pub;
  ros::Publisher m_drive_dir_pub;

  bool m_goal_received;
  bool m_task_is_new;
  bool m_use_de;

  UGV::NF1Evaluator::Target m_goal;
  UGV::UGVModel::State m_carrot;
  float m_ref_v, m_ref_w, m_ref_theta;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt, m_tht_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;
  std::vector<UGV::UGVModel::State> m_traj;
  bool cycle_initialized;
  int m_braking_start_cycle;
  int m_stuck_start_cycle;
  int m_full_start_cycle;
  int m_plan_request_cycle;
  int m_swarm_size;
  int m_batch_num;
  int m_episode_num;
  NF1MapDT *m_nf1_map;
  cpc_motion_planning::path m_stuck_recover_path;
  ros::ServiceClient m_collision_check_client;
  STUCK_SUB_MODE m_stuck_submode;

  //----------
  HomotopyClassPlannerPtr m_planner;
  TebConfig m_cfg;
  TebVisualizationPtr m_visualization;
  UGV::UGVModel::State m_ini_state;

};
}

#endif // UGV_TEB_PLANNER_H
