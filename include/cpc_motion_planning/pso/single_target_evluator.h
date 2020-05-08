#ifndef SINGLE_TARGET_EVLUATOR_H
#define SINGLE_TARGET_EVLUATOR_H
#include <cpc_motion_planning/pso/uav_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
namespace PSO
{
class SingleTargetEvaluator
{
public:

  struct Target
  {
    UAVModel::State s;
    bool oa;
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
  float process_cost(const UAVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;
    float3 dist_err = s.p - m_goal.s.p;
    cost += 0.5f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y + 3*dist_err.z*dist_err.z);
    cost += 0.1f*sqrt(s.v.x*s.v.x + s.v.y*s.v.y + s.v.z*s.v.z);
    cost += 0.1f*sqrt(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

    if (m_goal.oa)
    {
      int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
      int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
      int iz = floor( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5);

      if (ix<0 || ix>=map.m_map_size.x ||
          iy<0 || iy>=map.m_map_size.y ||
          iz<0 || iz>=map.m_map_size.z)
      {
        cost += 100;
        return cost;
      }

  #ifdef  __CUDA_ARCH__
      float rd = map.edt_const_at(ix,iy,iz).d*0.2;
      cost += exp(-6*rd)*400;

      if (rd < 0.6)
        cost += 100;

      if (!map.edt_const_at(ix,iy,iz).s)
          cost += 70;
  #endif
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UAVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;
    float3 dist_err = s.p - m_goal.s.p;
    cost += 1.0f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y + 3*dist_err.z*dist_err.z);
    //cost += 0.1f*sqrt(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

    if (m_goal.oa)
    {
      int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
      int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
      int iz = floor( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5);

      if (ix<0 || ix>=map.m_map_size.x ||
          iy<0 || iy>=map.m_map_size.y ||
          iz<0 || iz>=map.m_map_size.z)
      {
        cost += 100;
        return cost;
      }

  #ifdef  __CUDA_ARCH__
      float rd = map.edt_const_at(ix,iy,iz).d*0.2;
      cost += exp(-6*rd)*400;

      if (rd < 0.6)
        cost += 100;

      if (!map.edt_const_at(ix,iy,iz).s)
          cost += 70;
  #endif
    }

    return  cost;
  }

  Target m_goal;
};
}

#endif // SINGLE_TARGET_EVLUATOR_H
