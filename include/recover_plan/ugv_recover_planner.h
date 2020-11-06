#ifndef UGV_RECMOTION_PLANNER_H
#define UGV_RECMOTION_PLANNER_H
#include <loc_plan/ugv_base_local_planner.h>
#include <cpc_motion_planning/line_target.h>
#include <cpc_motion_planning/path.h>
#include <std_msgs/Int32.h>

#define RECOVER_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::RecoverEvaluator,UGV::UGVSwarm<3>
class UGVRecMotionPlanner
{
public:
  UGVRecMotionPlanner();
  ~UGVRecMotionPlanner();

  bool calculate_trajectory(const UGV::UGVModel::State &s, EDTMap *edt_map, std::vector<UGV::UGVModel::State> &traj);
  void set_path_cell(const cpc_motion_planning::path &path);

private:
  UGV::UGVModel::State calculate_tgt_state(const cpc_motion_planning::path_action &pa);
  UGV::UGVModel::State find_carrot(const cpc_motion_planning::path_action &pa);
  bool check_action_finish(const cpc_motion_planning::path_action &pa);

private:
  PSO::Planner<RECOVER_UGV> *m_pso_planner;
  cpc_motion_planning::path m_path;
  size_t m_curr_action_id;
  UGV::UGVModel::State m_tgt_state;
};

#endif // UGV_MOTION_PLANNER_H
