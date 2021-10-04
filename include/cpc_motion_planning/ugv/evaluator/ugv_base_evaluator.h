#ifndef UGV_BASE_EVALUATOR_H
#define UGV_BASE_EVALUATOR_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/ugv/evaluator/bilinear_interpolation.h>

namespace UGV
{
class BaseEvaluator
{
public:
  BaseEvaluator():m_footprint_offset(0.25f), m_safety_radius(0.401f)
  {

  }

  ~BaseEvaluator()
  {

  }

  __host__ __device__
  void calculate_bounding_centres(const UGVModel::State &s, float2 &c_r, float2 &c_f) const
  {
    float2 uni_dir = make_float2(cosf(s.theta),sinf(s.theta));
    c_f = s.p + m_footprint_offset*uni_dir;
    c_r = s.p - m_footprint_offset*uni_dir;
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

  float m_footprint_offset;
  float m_safety_radius;
};
}


#endif // UGV_BASE_EVALUATOR_H
