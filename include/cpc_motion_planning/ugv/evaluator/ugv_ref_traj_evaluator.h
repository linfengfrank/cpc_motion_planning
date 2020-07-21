#ifndef UGV_REF_TRAJ_EVALUATOR_H
#define UGV_REF_TRAJ_EVALUATOR_H
#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
namespace UGV
{
class RefTrajEvaluator
{
public:

  struct Ref
  {
    float3 tarj[41];
  };

  RefTrajEvaluator()
  {

  }

  ~RefTrajEvaluator()
  {

  }

  __host__ __device__
  void setRef(const Ref &ref)
  {
    m_ref = ref;
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
    int idx = min(40, static_cast<int>(floorf(time/0.1f + 0.5f)));
    float3 tgt = m_ref.tarj[idx];

    // Velocity cost
    float cost = 0.2f*sqrt(3.0f*s.v*s.v + s.w*s.w);

    // Horizontal dist cost
    cost += 0.5f*sqrt((s.p.x-tgt.x)*(s.p.x-tgt.x)+(s.p.y-tgt.y)*(s.p.y-tgt.y));

    // Yaw cost
    float yaw_diff = s.theta - tgt.z;
    yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
    cost += 0.2f*fabs(yaw_diff);

    // Collision cost
    float rd = getMinDist(s,map);
    cost += exp(-9.5f*rd)*400;
    if (rd < 0.31)
      cost += 100;
    if (rd < 0.11 && time < 1.5)
    {
      collision = true;
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float3 tgt = m_ref.tarj[20];

    // Velocity cost
    float cost = 0;

//    // Horizontal dist cost
//    cost += 2.5f*sqrt((s.p.x-tgt.x)*(s.p.x-tgt.x)+(s.p.y-tgt.y)*(s.p.y-tgt.y));

//    // Yaw cost
//    float yaw_diff = s.theta - tgt.z;
//    yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
//    cost += 1.0f*fabs(yaw_diff);

//    // Collision cost
////    float rd = getMinDist(s,map);
////    cost += exp(-9.5f*rd)*400;
////    if (rd < 0.31)
////      cost += 100;
////    if (rd < 0.11 && time < 1.5)
////    {
////      collision = true;
////    }

     return  cost;
  }

  Ref m_ref;
};
}
#endif // UGV_REF_TRAJ_EVALUATOR_H
