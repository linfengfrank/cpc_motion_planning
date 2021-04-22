#ifndef PP_EVALUATOR_H
#define PP_EVALUATOR_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/ugv/evaluator/bilinear_interpolation.h>

namespace UGV
{
class PPEvaluator
{
public:

  PPEvaluator()
  {
    m_target_received = false;
    m_safety_radius = 0.401f;
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
  void calculate_bounding_centres(const UGVModel::State &s, float2 &c_r, float2 &c_f) const
  {
    float2 uni_dir = make_float2(cosf(s.theta),sinf(s.theta));
    c_f = s.p + 0.2f*uni_dir;
    c_r = s.p - 0.2f*uni_dir;
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
  float m_safety_radius;
};
}

#endif // PP_EVALUATOR_H
