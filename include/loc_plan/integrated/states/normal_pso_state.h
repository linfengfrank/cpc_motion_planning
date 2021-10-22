#ifndef NORMAL_PSO_STATE_H
#define NORMAL_PSO_STATE_H

#include <loc_plan/integrated/state.h>
#include <iostream>
#include <ros/ros.h>
#include <cpc_motion_planning/ugv/swarm/ugv_swarm.h>
#include <cpc_motion_planning/ugv/controller/ugv_dp_control.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_nf1_evaluator.h>
#include <cpc_motion_planning/pso/pso_planner.h>

#define SIMPLE_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::NF1Evaluator,UGV::UGVSwarm<8>

class LocalPlannerPipeline;

// Every state has to be a singleton class
class NormalPsoState : public State
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

  // Use the PSO planner to calculate the trajectory
  void calculate_trajectory(EDTMap * edt_map,
                            std::vector<UGV::UGVModel::State> &traj,
                            bool is_stuck = false,
                            bool pure_de = false)
  {
    m_pso_planner->m_eva.m_stuck = is_stuck;
    // conduct the motion planning
    if (pure_de)
      m_pso_planner->plan_de(*edt_map);
    else
      m_pso_planner->plan(*edt_map);

    // generate the trajectory
    traj = m_pso_planner->generate_trajectory();
  }

  bool is_result_safe();

private:
  LocalPlannerPipeline *m_p = nullptr;
  ros::NodeHandle m_nh;
  bool m_plan_success = false;
  std::vector<UGV::UGVModel::State> m_pso_traj;

  PSO::Planner<SIMPLE_UGV> *m_pso_planner;
  bool m_use_de;
  int m_swarm_size;
  int m_batch_num;
  int m_episode_num;
  float m_footprint_offset; // The offset distance for the two point model

private:
  // private constructor, copy constructor and = operator for the singleton class
  NormalPsoState();
  NormalPsoState(const NormalPsoState& other);
  NormalPsoState& operator=(const NormalPsoState& other);

  void set_driving_dir();
  void set_exec_drive_direction();

private:
  // Proposition evaluation functions
  bool check_stuck();
  bool check_reach();
  bool check_success();

};

#endif // NORMAL_PSO_STATE_H
