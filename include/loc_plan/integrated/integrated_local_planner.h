#ifndef INTEGRATED_LOCAL_PLANNER_H
#define INTEGRATED_LOCAL_PLANNER_H
#include <loc_plan/ugv_base_local_planner.h>
#include <cpc_motion_planning/path.h>
#include <std_msgs/Int32.h>
#include <pure_pursuit/pure_pursuit_ctrl.h>
#include <cpc_aux_mapping/nf1_task.h>

#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>
#include <mpc/ltv_mpc_filter.h>

#define SIMPLE_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::NF1Evaluator,UGV::UGVSwarm<8>
//#define SHOW_INIT_PLAN
namespace teb = teb_local_planner;
class IntLocalPlanner : public UGVLocalMotionPlanner
{
  enum STUCK_SUB_MODE
  {
    RECOVER = 0,
    FULL_STUCK
  };

public:
  IntLocalPlanner();
  ~IntLocalPlanner();

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
  bool do_normal_teb();
  bool do_normal_pso();
private:
  void plan_call_back(const ros::TimerEvent&);
  void nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg);
  void cycle_init();
  std::vector<double2> get_init_path_guess();
  bool get_smallest_child(const CUDA_GEO::coord &c, CUDA_GEO::coord &bc);
  bool smooth_reference(const UGV::UGVModel::State &ini_state, const std::vector<teb::Reference> &raw_ref,
                        std::vector<UGV::UGVModel::State> &final_ref, bool use_simple_filter);
  void check_reach_and_stuck();
  // upate the vehicle's maximum speed with the current trajectory's minimum distance to obstacle
  void update_max_speed(const std::vector<UGV::UGVModel::State> &traj);

  bool check_tgt_is_same(const UGV::NF1Evaluator::Target &t1, const UGV::NF1Evaluator::Target &t2)
  {
    if (t1.act_id == t2.act_id && t1.path_id == t2.path_id)
      return true;
    else
      return false;
  }

  //---
  float acc_filter(float curr_v, float tgt_v, const float acc_lim, const float dt)
  {
    float achieved_v;
    float desired_acc = (tgt_v - curr_v)/dt;

    if(fabsf(desired_acc) < acc_lim)
    {
      achieved_v = tgt_v;
    }
    else
    {
      if (desired_acc > 0)
        achieved_v = curr_v + acc_lim*dt;
      else
        achieved_v = curr_v - acc_lim*dt;
    }
    return achieved_v;
  }

  int bool_to_drive_type(bool is_forward)
  {
    if (is_forward)
      return cpc_aux_mapping::nf1_task::TYPE_FORWARD;
    else
      return cpc_aux_mapping::nf1_task::TYPE_BACKWARD;
  }

  //---
  void publish_drive_direction_teb()
  {
    // For the TEB planner, its driving direction is pre-determined
    std_msgs::Int32 drive_dir;
    drive_dir.data = bool_to_drive_type(m_teb_planner->m_is_forward);
    m_drive_dir_pub.publish(drive_dir);
  }

  void publish_drive_direction_pso()
  {
    // For the PSO planner, its driving direction can be changed in the planner
    // if m_using_auto_direction is true
    std_msgs::Int32 drive_dir;

    // If auto_direction is true, we take the driving direction of
    // the first segment of PSO's trajectory
    // m_pso_planner->result.best_loc[0].z > 0 means forward, < 0 means backward.
    // If the auto_direction is false, then just use the  is_forward data
    // stored inside the evaluator (shall be assigned from the NF1 map).
    if(m_pso_planner->m_eva.m_using_auto_direction)
      drive_dir.data = bool_to_drive_type(m_pso_planner->result.best_loc[0].z > 0);
    else
      drive_dir.data = bool_to_drive_type(m_pso_planner->m_eva.is_forward);

    m_drive_dir_pub.publish(drive_dir);
  }

  bool is_tracking_safe(const nav_msgs::Odometry &odom, const std::vector<UGV::UGVModel::State> &ref)
  {
    // If not in ADRC mode (tracking mode), the simulated trajectory might collide with obstacle
    if(!m_use_adrc && tracking_min_edt(odom, ref, 2, m_turning_efficiency) < PSO::MIN_DIST)
      return false;
    else
      return true;
  }

private:
  ros::Subscriber m_nf1_sub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  ros::Publisher m_status_pub;
  ros::Publisher m_tgt_reached_pub;
  ros::Publisher m_drive_dir_pub;

  bool m_goal_received;
  bool m_task_is_new;
  bool m_use_de;
  PSO::Planner<SIMPLE_UGV> *m_pso_planner;
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
  int m_swarm_size;
  int m_batch_num;
  int m_episode_num;
  NF1MapDT *m_nf1_map;
  double m_turning_efficiency;
  bool m_use_adrc;
  bool m_allow_update_max_speed;

  //--- Teb planner
  teb::HomotopyClassPlannerPtr m_teb_planner;
  teb::TebConfig m_cfg;
  teb::TebVisualizationPtr m_visualization;

  //--- LTV acceleration filter
  int N_hor;
  ltv_mpc_filter* m_mpc;
  bool m_use_simple_filter;

#ifdef SHOW_INIT_PLAN
  ros::Publisher m_init_plan_pub;
  PointCloud::Ptr m_init_plan_cld;
#endif
};

#endif // INTEGRATED_LOCAL_PLANNER_H
