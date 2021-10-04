#ifndef UGV_REF_TRAJ_EVALUATOR_H
#define UGV_REF_TRAJ_EVALUATOR_H
#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/ugv/evaluator/ugv_base_evaluator.h>

namespace UGV
{
class RefTrajEvaluator : public BaseEvaluator
{
public:

  struct Target
  {
    UGVModel::State s;
    int id;
    Target():id(0)
    {

    }
  };

  RefTrajEvaluator()
  {
    m_pure_turning = false;
  }

  ~RefTrajEvaluator()
  {

  }

  __host__ __device__
  void setTarget(const Target &goal)
  {
    m_goal = goal;
  }

  __host__ __device__
  float process_cost(const UGVModel::State &s, const EDTMap &map, const float &time, PSO::EvaData &data) const
  {
    float cost = 0;

    // Collision cost
    float rd = getEDT(s.p,map);
    cost += expf(-8.5f*rd)*400;

    if (rd < 0.61f)
      cost += 100;

    if (rd < 0.21f && time < 1.5f)
    {
      data.collision = true;
    }

    if(!m_pure_turning)
    {
      //Distance cost
      float2 dist_err = m_goal.s.p - s.p;
      cost += 0.5f*sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + 0.2f*sqrtf(3.0f*s.v*s.v + 0.2*s.w*s.w);
      float yaw_diff = s.theta - m_goal.s.theta;
      yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
      cost += 0.4f*fabsf(yaw_diff);
    }
    else
    {
      //Pure heading cost
      cost += 5.0f*sqrtf(s.v*s.v); // stay still during turning
      float yaw_diff = s.theta - m_goal.s.theta;
      yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
      cost += 0.5f*fabsf(yaw_diff);
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;
    return  cost;
  }

  Target m_goal;
  bool m_pure_turning;
};
}
#endif // UGV_REF_TRAJ_EVALUATOR_H
