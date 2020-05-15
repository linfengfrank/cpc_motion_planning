#ifndef UGV_SINGLE_TARGET_EVALUATOR_H
#define UGV_SINGLE_TARGET_EVALUATOR_H

#include <cpc_motion_planning/ugv/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
namespace UGV
{
class SingleTargetEvaluator
{
public:

  struct Target
  {
    UGVModel::State s;
  };

  SingleTargetEvaluator()
  {

  }

  ~SingleTargetEvaluator()
  {

  }

  __host__ __device__
  void setTarget(const Target &goal)
  {
    m_goal = goal;
  }

  __host__ __device__
  float process_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;
    float2 dist_err = s.p - m_goal.s.p;
    cost += 0.5f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + 0.2f*sqrt(s.v*s.v + s.w*s.w);

    int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
    int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
    int iz = floor( (0.35 - map.m_origin.z) / map.m_grid_step + 0.5);

    if (ix<0 || ix>=map.m_map_size.x ||
            iy<0 || iy>=map.m_map_size.y ||
            iz<0 || iz>=map.m_map_size.z)
    {
        cost += 100;
        return cost;
    }

  #ifdef  __CUDA_ARCH__
    float rd = map.edt_const_at(ix,iy,iz).d*0.2;
    cost += exp(-4.5f*rd)*400;

    if (rd < 1.0)
      cost += 100;
  #endif

    if (sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) > 1)
    {
      cost += 0.5f*M_PI;
    }
    else
    {
      float yaw_diff = s.theta - m_goal.s.theta;
      yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
      cost += 0.5f*fabs(yaw_diff);
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;
     float2 dist_err = s.p - m_goal.s.p;
     cost += 4.0f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + 0.2f*sqrt(s.v*s.v + s.w*s.w);

     int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
     int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
     int iz = floor( (0.35 - map.m_origin.z) / map.m_grid_step + 0.5);

     if (ix<0 || ix>=map.m_map_size.x ||
             iy<0 || iy>=map.m_map_size.y ||
             iz<0 || iz>=map.m_map_size.z)
     {
         cost += 100;
         return cost;
     }

   #ifdef  __CUDA_ARCH__
     float rd = map.edt_const_at(ix,iy,iz).d*0.2;
     cost += exp(-4.5f*rd)*400;

     if (rd < 1.0)
       cost += 100;
   #endif

     if (sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) > 1)
     {
       cost += 4*M_PI;
     }
     else
     {
       float yaw_diff = s.theta - m_goal.s.theta;
       yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
       cost += 4*fabs(yaw_diff);
     }

     return  cost;
  }

  Target m_goal;
};
}

#endif // UGV_SINGLE_TARGET_EVALUATOR_H
