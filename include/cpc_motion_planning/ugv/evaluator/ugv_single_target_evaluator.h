#ifndef UGV_SINGLE_TARGET_EVALUATOR_H
#define UGV_SINGLE_TARGET_EVALUATOR_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1map.cuh>
namespace UGV
{
class SingleTargetEvaluator
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

  SingleTargetEvaluator()
  {
    m_pure_turning = false;
    m_nf1_received = false;
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
  float2 get_head_pos(const UGVModel::State &s,float r) const
  {
    float2 head_pos;
    head_pos.x = s.p.x + r*cosf(s.theta);
    head_pos.y = s.p.y + r*sinf(s.theta);
    return head_pos;
  }

  __host__ __device__
  float process_cost(const UGVModel::State &s, const EDTMap &map, const float &time, bool &collision) const
  {
    float cost = 0;

    // Collision cost
    float rd = getEDT(s.p,map);
    cost += expf(-8.5f*rd)*200;

    if (rd < 0.61f)
      cost += 100;

    if (rd < 0.21f && time < 1.5f)
    {
      collision = true;
    }

    if(!m_pure_turning)
    {
      //Distance cost
      if (!m_nf1_received)
      {
        float2 dist_err = s.p - m_goal.s.p;
        cost += 0.5f*sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + 0.2f*sqrtf(3.0f*s.v*s.v + 0.2*s.w*s.w);
      }
      else
      {
        float2 head_pos = get_head_pos(s,0.3f);
        CUDA_GEO::coord head_c = m_nf1_map.pos2coord(make_float3(head_pos.x,head_pos.y,0));
        CUDA_GEO::coord c = m_nf1_map.pos2coord(make_float3(s.p.x,s.p.y,0));
#ifdef  __CUDA_ARCH__
        // Must use c.x c.y and 0 here! Because the NF1 map has only 1 layer.
        cost += 1.0f*m_nf1_map.nf1_const_at(head_c.x,head_c.y,0);
#endif
        float yaw_diff = s.theta - getDesiredHeading(c);
        yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
        cost += yaw_diff*yaw_diff*s.v*s.v;
      }
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

    if(!m_pure_turning && m_nf1_received)
    {
      float2 head_pos = get_head_pos(s,0.3f);
      CUDA_GEO::coord head_c = m_nf1_map.pos2coord(make_float3(head_pos.x,head_pos.y,0));
#ifdef  __CUDA_ARCH__
      // Must use c.x c.y and 0 here! Because the NF1 map has only 1 layer.
      cost += 20.0f*m_nf1_map.nf1_const_at(head_c.x,head_c.y,0);
#endif
    }

    return  cost;
  }

  Target m_goal;
  bool m_pure_turning;
  NF1Map m_nf1_map;
  bool m_nf1_received;
};
}

#endif // UGV_SINGLE_TARGET_EVALUATOR_H
