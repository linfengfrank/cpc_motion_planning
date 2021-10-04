#ifndef PP_EVALUATOR_H
#define PP_EVALUATOR_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/ugv/evaluator/bilinear_interpolation.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_base_evaluator.h>

namespace UGV
{
class PPEvaluator : public BaseEvaluator
{
public:

  PPEvaluator()
  {
    m_target_received = false;
  }

  ~PPEvaluator()
  {

  }

  __host__ __device__
  void setGoal(const UGVModel::State &goal)
  {
    m_goal = goal;
  }

  __host__ __device__
  void setCarrot(const UGVModel::State &carrot)
  {
    m_carrot = carrot;
  }

  __host__ __device__
  float process_cost(const UGVModel::State &s, const EDTMap &map, const float &time, PSO::EvaData &data) const
  {
    float cost = 0;

    float rd = getMinDist(s,map);
    cost += expf(-9.5f*rd)*10;

    if (rd < m_safety_radius)
      cost += (1000 + expf(-4.5f*rd)*1000);

    if (rd < PSO::MIN_DIST && time < 0.31f)
    {
      data.collision = true;
    }

    data.min_dist = rd;

    if (m_target_received)
    {
      // distance to goal
      float2 dist_err = m_carrot.p - s.p;
      float dist_val = sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y);
      cost += 0.4f*dist_val;// + sqrtf(0.0f*s.v*s.v + 0.0f*s.w*s.w);

      // desired angle
      float2 dist_err_g = m_goal.p - s.p;
      float dist_val_g = sqrtf(dist_err_g.x*dist_err_g.x + dist_err_g.y*dist_err_g.y);
      float yaw_gain = dist_val_g*dist_val_g;
      if (yaw_gain > 1.0f) yaw_gain = 1.0f;
      float yaw_diff = s.theta - m_carrot.theta;
      yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
      cost += (1.0f+2.0f*s.v*s.v)*yaw_gain*yaw_diff*yaw_diff;

      // energy cost
      float gain = fabsf(data.current_v);
      if (gain < 0.1f)
        gain = 0;
      if (gain > 1.0f)
        gain = 1.0f;
      cost += 1.0f*sqrtf(0.005f*s.v*s.v + 1.0f*s.w*s.w*gain);
    }
    else
    {
      // have not received the guidance function map yet, stay still
      cost += 1.0f*sqrtf(s.v*s.v + s.w*s.w);
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;

    float2 dist_err = m_carrot.p - s.p;
    float dist_val = sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y);
    cost += 20.0f*dist_val;

    return  cost;
  }

  UGVModel::State m_goal;
  UGVModel::State m_carrot;
  bool m_target_received;
};
}

#endif // PP_EVALUATOR_H
