#ifndef UAV_NF1_EVLUATOR_H
#define UAV_NF1_EVLUATOR_H
#include <cpc_motion_planning/uav/uav_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_nf1map.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
namespace UAV
{
class NF1Evaluator
{
public:

  struct Target
  {
    bool oa;
  };

  NF1Evaluator():m_oa(false)
  {

  }

  ~NF1Evaluator()
  {

  }

  __host__ __device__
  float process_cost(const UAVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;
    float dist_err = s.p.z - 2.0f;
    cost += 0.5f*3.0f*sqrtf(dist_err*dist_err);
    cost += 0.1f*sqrtf(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

   CUDA_GEO::coord c = m_nf1_map.pos2coord(s.p);
#ifdef  __CUDA_ARCH__
    cost += 0.5f*m_nf1_map.nf1_const_at(c.x,c.y,0);
#endif
        //cost += 0.5f*1.0f*sqrtf(s.p.x*s.p.x + s.p.y+s.p.y);

    if (m_oa)
    {
      int ix = floorf( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
      int iy = floorf( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
      int iz = floorf( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5);

      if (ix<0 || ix>=map.m_map_size.x ||
          iy<0 || iy>=map.m_map_size.y ||
          iz<0 || iz>=map.m_map_size.z)
      {
        cost += 100;
        return cost;
      }

  #ifdef  __CUDA_ARCH__
      float rd = map.edt_const_at(ix,iy,iz).d*0.2;
      cost += expf(-6*rd)*400;

      if (rd < 0.6)
        cost += 100;

      if (!map.edt_const_at(ix,iy,iz).s)
          cost += 100;
  #endif
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UAVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;
    float dist_err = s.p.z - 2.0f;
    cost += 0.5f*3.0f*sqrtf(dist_err*dist_err);
    cost += 0.1f*sqrtf(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

    CUDA_GEO::coord c = m_nf1_map.pos2coord(s.p);
#ifdef  __CUDA_ARCH__
    cost += 0.5f*m_nf1_map.nf1_const_at(c.x,c.y,0);
#endif
    //cost += 0.5f*1.0f*sqrtf(s.p.x*s.p.x + s.p.y+s.p.y);


    if (m_oa)
    {
      int ix = floorf( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
      int iy = floorf( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
      int iz = floorf( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5);

      if (ix<0 || ix>=map.m_map_size.x ||
          iy<0 || iy>=map.m_map_size.y ||
          iz<0 || iz>=map.m_map_size.z)
      {
        cost += 100;
        return cost;
      }

  #ifdef  __CUDA_ARCH__
      float rd = map.edt_const_at(ix,iy,iz).d*0.2;
      cost += expf(-6*rd)*400;

      if (rd < 0.6)
        cost += 100;

      if (!map.edt_const_at(ix,iy,iz).s)
          cost += 100;
  #endif

      float3 diff = s.p - m_curr_pos;
      diff.z = 0;
      if (sqrtf(diff.x*diff.x + diff.y*diff.y) > 0.3f)
      {
          float theta = atan2f(diff.y,diff.x);
          theta -= m_curr_yaw;
          theta = theta - floorf((theta + M_PI) / (2 * M_PI)) * 2 * M_PI;
          if (fabsf(theta) > M_PI*0.25f)
              cost += 100;
      }
    }

    return  cost;
  }
  bool m_oa;
  float m_curr_yaw;
  float3 m_curr_pos;
  NF1Map m_nf1_map;
};
}
#endif // UAV_NF1_EVLUATOR_H
