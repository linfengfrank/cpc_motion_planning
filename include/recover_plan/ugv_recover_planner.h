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
  UGV::UGVModel::State find_carrot(const cpc_motion_planning::path_action &pa, float2 &line_a, float2 &line_b);
  float find_turning_angle(const cpc_motion_planning::path_action &pa);
  bool check_action_finish(const cpc_motion_planning::path_action &pa);
  bool is_curvature_too_big(const cpc_motion_planning::path_action &pa, size_t start, size_t end);

  float in_pi(float in)
  {
    return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
  }

  inline float pnt2line_dist(const float3 & c1, const float3 & c2, const float3 & c0, float scale_verticle = 1.0f)
  {
    float3 a = c1-c0;
    float3 b = c2-c1;

    a.z = a.z * scale_verticle;
    b.z = b.z * scale_verticle;

    float a_square = dot(a,a);
    float b_square = dot(b,b);
    float a_dot_b = a.x*b.x + a.y*b.y + a.z*b.z;

    if (b_square < 1e-3)
      return sqrtf(static_cast<float>(a_square));

    return sqrtf(static_cast<float>(a_square*b_square - a_dot_b*a_dot_b)/static_cast<float>(b_square));
  }

private:
  PSO::Planner<RECOVER_UGV> *m_pso_planner;
  cpc_motion_planning::path m_path;
  size_t m_curr_action_id;
  UGV::UGVModel::State m_tgt_state;
  ros::Publisher m_carrot_pub;
};

#endif // UGV_MOTION_PLANNER_H
