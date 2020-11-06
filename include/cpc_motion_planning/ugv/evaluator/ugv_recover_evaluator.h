#ifndef UGV_RECOVER_EVALUATOR_H
#define UGV_RECOVER_EVALUATOR_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/ugv/evaluator/bilinear_interpolation.h>

namespace UGV
{
class RecoverEvaluator
{
public:

  //  struct Target
  //  {
  //    UGVModel::State s;
  //  };

  RecoverEvaluator()
  {
    m_mode = 0;
  }

  ~RecoverEvaluator()
  {

  }

  __host__ __device__
  void setTarget(const UGVModel::State &goal)
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

    float rd = getMinDist(s,map);
    cost += expf(-9.5f*rd)*10;

    if (rd < 0.41f)
      cost += 1000;

    if (rd < 0.23f && time < 0.4f)
    {
      collision = true;
    }

    switch (m_mode)
    {
    case 0: // don't move
      cost += 1.0f*sqrtf(s.v*s.v + s.w*s.w);
      break;
      //---
    case 1: // spot turn
    {
      float2 dist_err = m_goal.p - s.p;
      cost += 0.5f*sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + sqrtf(0.1f*s.v*s.v + 0.0f*s.w*s.w);

      float yaw_diff = s.theta - m_goal.theta;
      yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
      cost += 0.5f*fabsf(yaw_diff);

      break;
    }
      //---
    case 2:
    {
      float2 dist_err = m_goal.p - s.p;
      float dist_val = sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y);
      cost += 0.5f*dist_val + sqrtf(0.1f*s.v*s.v + 0.1f*s.w*s.w);

      float yaw_diff = s.theta - m_goal.theta;
      yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;

      float yaw_gain = dist_val*dist_val;
      if (yaw_gain > 1.0f) yaw_gain = 1.0f;
      cost += 0.5f*yaw_gain*fabsf(yaw_diff);

      break;
    }
      //---
    case 3:
    {
      float2 dist_err = m_goal.p - s.p;
      float dist_val = sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y);
      cost += 0.5f*dist_val + sqrtf(0.1f*s.v*s.v + 0.1f*s.w*s.w);

      float yaw_diff = s.theta - m_goal.theta;
      yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;

      float yaw_gain = dist_val*dist_val;
      if (yaw_gain > 1.0f) yaw_gain = 1.0f;
      cost += 0.5f*yaw_gain*fabsf(yaw_diff);

      break;
    }
      //---
    default:
    {
      cost += 1.0f*sqrtf(s.v*s.v + s.w*s.w);
    }
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;

    return  cost;
  }

  UGVModel::State m_goal;
  //0:nothing, 1:turning, 2:forward, 3:backward
  unsigned char m_mode;
};
}

#endif // UGV_RECOVER_EVALUATOR_H
