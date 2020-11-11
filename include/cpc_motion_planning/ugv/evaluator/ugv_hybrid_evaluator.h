#ifndef UGV_HYBRID_EVALUATOR_H
#define UGV_HYBRID_EVALUATOR_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1map.cuh>

#define THETA_GRID_SIZE 24
namespace UGV
{
class HybridEvaluator
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

  HybridEvaluator()
  {
    m_pure_turning = false;
    m_nf1_received = false;
    m_stuck = false;
  }

  ~HybridEvaluator()
  {

  }

  __host__ __device__
  void setTarget(const Target &goal)
  {
    m_goal = goal;
  }

  __host__ __device__
  void calculate_bounding_centres(const UGVModel::State &s, float2 &c_r, float2 &c_f) const
  {
    float2 uni_dir = make_float2(cosf(s.theta),sinf(s.theta));
    c_f = s.p + 0.25f*uni_dir;
    c_r = s.p - 0.25f*uni_dir;
  }

  __host__ __device__
  float getEDT(const float2 &p, const EDTMap &map) const
  {
    int ix = floorf( (p.x - map.m_origin.x) / map.m_grid_step + 0.5f);
    int iy = floorf( (p.y - map.m_origin.y) / map.m_grid_step + 0.5f);
    int iz = floorf( (0 - map.m_origin.z) / map.m_grid_step + 0.5f);
    if (ix<0 || ix>=map.m_map_size.x ||
        iy<0 || iy>=map.m_map_size.y ||
        iz<0 || iz>=map.m_map_size.z)
    {
      return 0;
    }
    else
    {
#ifdef  __CUDA_ARCH__
      return map.edt_const_at(ix,iy,iz).d * map.m_grid_step;
#else
      return 0;
#endif
    }
  }

  __host__ __device__
  float getMinDist(const UGVModel::State &s, const EDTMap &map) const
  {
    float2 c_f,c_r;
    calculate_bounding_centres(s, c_r, c_f);
    return min(getEDT(c_r,map),getEDT(c_f,map));
  }

  __host__ __device__
  float grid2theta(int grid) const
  {
    grid = positive_modulo(grid,THETA_GRID_SIZE);
    return 2.0f*static_cast<float>(M_PI)/static_cast<float>(THETA_GRID_SIZE)*static_cast<float>(grid);
  }

  __host__ __device__
  int theta2grid(float theta) const
  {
    int grid = floor(theta/(2.0f*static_cast<float>(M_PI)/static_cast<float>(THETA_GRID_SIZE)) + 0.5);
    return positive_modulo(grid,THETA_GRID_SIZE);
  }

  __host__ __device__
  float getDesiredHeading(const CUDA_GEO::coord &c) const
  {
    float min_cost = 1e6;
    float cost = 0;
    float2 dir = make_float2(0,0);
#ifdef  __CUDA_ARCH__
    for (int x=-1;x<=1;x++)
    {
      for (int y=-1;y<=1;y++)
      {
        cost = m_nf1_map.nf1_const_at(c.x+x,c.y+y,0);
        if (cost < min_cost)
        {
          min_cost = cost;
          dir = make_float2(x,y);
        }
      }
    }
#endif
    return atan2f(dir.y,dir.x);
  }

  __host__ __device__
  int positive_modulo(int i, int n) const
  {
    return (i % n + n) % n;
  }

  __host__ __device__
  float process_cost(const UGVModel::State &s, const EDTMap &map, const float &time, bool &collision) const
  {
    float cost = 0;

    // Collision cost
    float rd = getMinDist(s,map);
    cost += expf(-10.5f*rd)*50;

    if (rd < 0.51f)
      cost += 1000;

    if (rd < 0.23f && time < 0.4f)
    {
      collision = true;
    }

    if (m_nf1_received)
    {
      if (m_pure_turning)
      {
        //reached target, pure turning mode
        cost += 5.0f*sqrtf(s.v*s.v); // stay still during turning
        float yaw_diff = s.theta - m_goal.s.theta;
        yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
        cost += 0.5f*fabsf(yaw_diff);
      }
      else
      {
        //either stuck or normal mode
        CUDA_GEO::coord c = m_nf1_map.pos2coord(make_float3(s.p.x,s.p.y,0));
        c.z = theta2grid(s.theta);
        float nf_cost = 0;
#ifdef  __CUDA_ARCH__
        nf_cost = 0.5f*m_nf1_map.nf1_const_at(c.x,c.y,c.z);
#endif
        if (m_stuck)
        {
          //stuck mode, encourage random move to get out of stuck
          cost += 2.0f*fabsf(fabsf(s.v) - 0.3f) + 0.02f*nf_cost;
        }
        else
        {
          //normal mode
          cost += 0.5f*nf_cost + 0.01f*sqrtf(0.1f*s.v*s.v + 0.1f*s.w*s.w);
        }
      }
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

    return  cost;
  }

  Target m_goal;
  bool m_pure_turning;
  bool m_stuck;
  NF1Map m_nf1_map;
  bool m_nf1_received;
};
}

#endif // UGV_HYBRID_EVALUATOR_H
