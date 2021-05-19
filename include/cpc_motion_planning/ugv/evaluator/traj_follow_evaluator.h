#ifndef TRAJ_FOLLOW_EVALUATOR_H
#define TRAJ_FOLLOW_EVALUATOR_H
#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/ugv/evaluator/bilinear_interpolation.h>

namespace UGV
{
class TFEvaluator
{
public:

  struct Target
  {
    UGVModel::State s;
    int path_id;
    int act_id;
    float reaching_radius;
    Target():path_id(-1),act_id(-1),reaching_radius(1.0f)
    {

    }
  };

  TFEvaluator()
  {
    m_accurate_reaching = false;
    is_forward = true;
    m_pure_turning = false;
    m_nf1_received = false;
    m_stuck = false;
    m_using_auto_direction = false;
    m_safety_radius = 0.401f;
  }

  ~TFEvaluator()
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

    if (m_using_auto_direction)
      is_forward = data.is_forward;

    float rd = getMinDist(s,map);
    //cost += expf(-9.5f*rd)*10;

    if (rd < m_safety_radius)
      cost += (1000 + expf(-4.5f*rd)*1000);

    if (rd < PSO::MIN_DIST && time < 0.31f)
    {
      data.collision = true;
    }

    data.min_dist = rd;

    // Get the time
    int idx = time/PSO::PSO_CTRL_DT;
    if (idx <0 || idx >=40)
      return 0;

    float3 ref = m_ref[idx];
    //position diff
    cost += 1.0*((s.p.x - ref.x)*(s.p.x - ref.x) + (s.p.y - ref.y)*(s.p.y - ref.y));

    //yaw angle diff
    float yaw_diff = s.theta - ref.z;
    yaw_diff = yaw_diff - floorf((yaw_diff + CUDA_F_PI) / (2 * CUDA_F_PI)) * 2 * CUDA_F_PI;
    cost += 0.5f*yaw_diff*yaw_diff;

    //control cost
    cost += 0.2f*s.w*s.w;

    return  cost;
  }

  __host__ __device__
  float final_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;

    return  cost;
  }

  bool m_accurate_reaching;
  Target m_goal;
  mutable bool is_forward;
  bool m_pure_turning;
  bool m_stuck;
  float3 m_ref[40];
  bool m_nf1_received;
  bool m_using_auto_direction;
  float m_safety_radius;
};
}

#endif // TRAJ_FOLLOW_EVALUATOR_H
