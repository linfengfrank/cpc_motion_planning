#ifndef NOMARL_STATE_H
#define NOMARL_STATE_H
#include <cpc_motion_planning/state.h>
#include <iostream>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>
#include <mpc/ltv_mpc_filter.h>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>

class LocalPlannerPipeline;

// Every state has to be a singleton class
namespace teb = teb_local_planner;
class NormalTebState : public State
{
  // Propositoin enum
  // must start with 0 and end with the MAX_SIZE
  // cannot assign custom values
  enum PPS
  {
    SUCCESS = 0,
    STUCK,
    REACH,
    MAX_SIZE
  };
public:
  void on_execute() override;
  void on_finish() override;
  State& toggle() override;
  void check_props() override;
  static State & getInstance(); // Get instance of the singleton class
  void attach_to_pipe(Pipeline *p) override;

  void on_activation() override {}
  void on_deactivation() override {}

private:
  LocalPlannerPipeline *m_p = nullptr;
  ros::NodeHandle m_nh;
  bool m_plan_success = false;
  std::vector<UGV::UGVModel::State> m_teb_traj;
  //--- Teb planner
  teb::HomotopyClassPlannerPtr m_teb_planner;
  teb::TebConfig m_cfg;
  teb::TebVisualizationPtr m_visualization;
  //--- LTV acceleration filter
  int N_hor;
  ltv_mpc_filter* m_mpc;
  bool m_use_simple_filter = true;



private:
  // private constructor, copy constructor and = operator for the singleton class
  NormalTebState();
  NormalTebState(const NormalTebState& other);
  NormalTebState& operator=(const NormalTebState& other);
  std::vector<double2> get_init_path_guess();
  bool get_smallest_child(NF1MapDT *nf1_map, const CUDA_GEO::coord &c, CUDA_GEO::coord& bc);
  bool smooth_reference(const UGV::UGVModel::State &ini_state, const std::vector<teb::Reference> &raw_ref,
                        std::vector<UGV::UGVModel::State> &final_ref);
  float acc_filter(float curr_v, float tgt_v, const float acc_lim, const float dt);
  void set_driving_dir();
  void set_exec_drive_direction();

private:
  // Proposition evaluation functions
  bool check_stuck();
  bool check_reach();
  bool check_success();
};

#endif // NOMARL_STATE_H
