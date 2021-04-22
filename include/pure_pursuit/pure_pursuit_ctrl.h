#ifndef UGV_RECMOTION_PLANNER_H
#define UGV_RECMOTION_PLANNER_H
#include <loc_plan/ugv_base_local_planner.h>
#include <cpc_motion_planning/line_target.h>
#include <cpc_motion_planning/path.h>
#include <std_msgs/Int32.h>

#define RECOVER_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::PPEvaluator,UGV::UGVSwarm<8>
class UGVRecMotionPlanner
{
public:
  UGVRecMotionPlanner();
  ~UGVRecMotionPlanner();

  void init_swarm(int step_num, float step_dt, float var_s, float var_theta, std::string file_location);
  bool calculate_trajectory(const UGV::UGVModel::State &s, UGV::UGVModel::State g,
                            EDTMap *edt_map, std::vector<UGV::UGVModel::State> &traj);

private:
  PSO::Planner<RECOVER_UGV> *m_pso_planner;
  cpc_motion_planning::path m_path;
  size_t m_curr_action_id;
  UGV::UGVModel::State m_tgt_state;
  ros::Publisher m_carrot_pub;
  cpc_motion_planning::path_action collision_checking_path;
};

#endif // UGV_MOTION_PLANNER_H
