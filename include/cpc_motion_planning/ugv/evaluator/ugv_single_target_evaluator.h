#ifndef UGV_SINGLE_TARGET_EVALUATOR_H
#define UGV_SINGLE_TARGET_EVALUATOR_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
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
  float process_cost(const UGVModel::State &s, const EDTMap &map, const float &time, bool &collision) const
  {
    float cost = 0;

    // Collision cost
    float rd = getMinDist(s,map);
    cost += exp(-9.5f*rd)*400;

    if (rd < 0.31)
      cost += 100;

    if (rd < 0.11 && time < 1.5)
    {
      collision = true;
    }

    if(!m_pure_turning)
    {
      //Distance cost
      float2 dist_err = s.p - m_goal.s.p;
      cost += 0.5f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + 0.2f*sqrt(3.0f*s.v*s.v + s.w*s.w);
    }
    else
    {
      //Pure heading cost
      cost += 5.0f*sqrt(s.v*s.v); // stay still during turning
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
//     float2 dist_err = s.p - m_goal.s.p;
//     cost += 0.5f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + 1.2f*sqrt(3.0f*s.v*s.v + s.w*s.w);

//     int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
//     int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
//     int iz = floor( (0 - map.m_origin.z) / map.m_grid_step + 0.5);

//     if (ix<0 || ix>=map.m_map_size.x ||
//             iy<0 || iy>=map.m_map_size.y ||
//             iz<0 || iz>=map.m_map_size.z)
//     {
//         cost += 100;
//         return cost;
//     }

//   #ifdef  __CUDA_ARCH__
//     float rd = map.edt_const_at(ix,iy,iz).d*map.m_grid_step;
//     cost += exp(-4.5f*rd)*400;

//     if (rd < 0.71)
//       cost += 100;
//   #endif

//     if (sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) > 1)
//     {
//       cost += 0.5f*M_PI;
//     }
//     else
//     {
//       float yaw_diff = s.theta - m_goal.s.theta;
//       yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
//       cost += 0.5f*fabs(yaw_diff);
//     }

     return  cost;
  }

  Target m_goal;
  bool m_pure_turning;
};
}

#endif // UGV_SINGLE_TARGET_EVALUATOR_H
