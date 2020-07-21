#ifndef UGV_CORRIDOR_EVALUATOR_H
#define UGV_CORRIDOR_EVALUATOR_H
#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>

namespace UGV
{
class CorridorEvaluator
{
public:
  struct Corridor
  {
    float3 a;
    float3 b;
    float l;
    float r;

    Corridor()
    {

    }

    void set_data(const float3 &a_, const float3 &b_, const float r_)
    {
      a = a_;
      b = b_;
      l = sqrtf(dot(a-b,a-b));
      r = r_;
    }
  };

  struct Target
  {
    Corridor* c;
    int n;
    bool oa;
  };

  CorridorEvaluator()
  {

  }

  ~CorridorEvaluator()
  {

  }

  __host__ __device__
  void linecirc_inter_dist(const float3 &seg_a, const float3 &seg_b, const float3 &circ_pos, float3 &closest, float &dist_v_len) const
  {
    float3 seg_v=seg_b-seg_a;
    float3 pt_v=circ_pos-seg_a;
    float seg_v_len = sqrtf(dot(seg_v,seg_v));
    float3 seg_v_unit=seg_v/seg_v_len;
    float proj=dot(pt_v,seg_v_unit);
    float3 proj_v=seg_v_unit*proj;
    if (proj <=0)
      closest =seg_a;
    else if(proj>seg_v_len)
      closest =seg_b;
    else
      closest=proj_v+seg_a;

    dist_v_len = sqrtf(dot(circ_pos-closest,circ_pos-closest));
  }

  __host__ __device__
  void find_nearest_point(const float3 &p, float & min_len, int &min_idx, float3 &nearest_point) const
  {
    min_len = 1e6;
    min_idx = 1;
    float3 pnt;
    float len;
    for (int i=0;i<m_sfc.n;i++)
    {
      linecirc_inter_dist(m_sfc.c[i].a,m_sfc.c[i].b,p,pnt,len);
      if (len < min_len)
      {
        min_len = len;
        min_idx = i;
        nearest_point = pnt;
      }
    }
  }

  __host__ __device__
  float corridor_cost(const Corridor &c, const int &c_idx , const float3 &p) const
  {
    float3 pnt;
    float len;
    linecirc_inter_dist(c.a,c.b,p,pnt,len);

    float cost = 0;
    for(int i=c_idx;i<m_sfc.n;i++)
    {
      if (i == c_idx)
        cost += sqrtf(dot(pnt-c.b, pnt-c.b));
      else
        cost += m_sfc.c[i].l;
    }

    cost += len;

    if (len > c.r)
      cost += 10 + 1e4f*(len -  c.r);

    return cost;
  }

  __host__ __device__
  float fc_cost(const float3 &p) const
  {
    float min_cost = 1e6;
    float cost;
    for(int i=0;i<m_sfc.n;i++)
    {
      cost = corridor_cost(m_sfc.c[i],i,p);
      if (cost < min_cost)
        min_cost = cost;
    }

    return min_cost;
  }

  __host__ __device__
  float process_cost(const UGVModel::State &s, const EDTMap &map, const float &time, bool &collision) const
  {
    float cost = 0;
    cost = cost+fc_cost(make_float3(s.p.x,s.p.y,0));
    //    float3 dist_err = s.p - m_goal.s.p;
    //    cost += 0.5f*sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y + 3*dist_err.z*dist_err.z);
    //    cost += 0.1f*sqrtf(s.v.x*s.v.x + s.v.y*s.v.y + s.v.z*s.v.z);
    //    cost += 0.1f*sqrtf(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

    //    if (m_goal.oa)
    //    {
    //      int ix = floorf( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
    //      int iy = floorf( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
    //      int iz = floorf( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5);

    //      if (ix<0 || ix>=map.m_map_size.x ||
    //          iy<0 || iy>=map.m_map_size.y ||
    //          iz<0 || iz>=map.m_map_size.z)
    //      {
    //        cost += 100;
    //        return cost;
    //      }

    //  #ifdef  __CUDA_ARCH__
    //      float rd = map.edt_const_at(ix,iy,iz).d*0.2;
    //      cost += expf(-6*rd)*400;

    //      if (rd < 0.6)
    //        cost += 100;

    //      if (!map.edt_const_at(ix,iy,iz).s)
    //          cost += 70;
    //  #endif
    //    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;
    //cost = cost+fc_cost(make_float3(s.p.x,s.p.y,0));
    //    float3 dist_err = s.p - m_goal.s.p;
    //    cost += 1.0f*sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y + 3*dist_err.z*dist_err.z);
    //    //cost += 0.1f*sqrtf(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

    //    if (m_goal.oa)
    //    {
    //      int ix = floorf( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
    //      int iy = floorf( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
    //      int iz = floorf( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5);

    //      if (ix<0 || ix>=map.m_map_size.x ||
    //          iy<0 || iy>=map.m_map_size.y ||
    //          iz<0 || iz>=map.m_map_size.z)
    //      {
    //        cost += 100;
    //        return cost;
    //      }

    //  #ifdef  __CUDA_ARCH__
    //      float rd = map.edt_const_at(ix,iy,iz).d*0.2;
    //      cost += expf(-6*rd)*400;

    //      if (rd < 0.6)
    //        cost += 100;

    //      if (!map.edt_const_at(ix,iy,iz).s)
    //          cost += 70;
    //  #endif

    //      float3 diff = s.p - m_curr_pos;
    //      diff.z = 0;
    //      if (sqrtf(diff.x*diff.x + diff.y*diff.y) > 0.3f)
    //      {
    //          float theta = atan2f(diff.y,diff.x);
    //          theta -= m_curr_yaw;
    //          theta = theta - floorf((theta + M_PI) / (2 * M_PI)) * 2 * M_PI;
    //          if (fabsf(theta) > M_PI*0.25f)
    //              cost += 100;
    //      }
    //    }

    return  cost;
  }

  float m_curr_yaw;
  float3 m_curr_pos;
  Target m_sfc;
};
}

#endif // UGV_CORRIDOR_EVALUATOR_H
