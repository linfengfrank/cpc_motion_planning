#ifndef BILINEAR_INTERPOLATION_H
#define BILINEAR_INTERPOLATION_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>

namespace UGV
{

//__host__ __device__ __forceinline__
//void get_opposite_pnt(int *loc, int *opp, const int *idx, int size)
//{
//  for (int i=0;i<size;i++)
//  {
//    if (loc[i] == 0)
//    {
//      opp[i] = 1 + idx[i];
//    }
//    else
//    {
//      opp[i] = 0 + idx[i];
//    }
//  }
//}


__host__ __device__ __forceinline__
float calculate_weight(const float2 &p, const int2 &corner)
{
  return (1-fabsf(p.x - corner.x))*(1-fabsf(p.y - corner.y));
}

__host__ __device__ __forceinline__
float bilinear_theta(const float2 &p, const NF1MapDT &map)
{

  // 1. Regulate the value (so now the width of a grid is 1)
  float2 p_r;
  p_r.x = (p.x - map.m_origin.x) / map.m_grid_step;
  p_r.y = (p.y - map.m_origin.y) / map.m_grid_step;

  // 2. Get the left bottom corner index
  int2 lb_idx;
  lb_idx.x = static_cast<int>(floorf(p_r.x));
  lb_idx.y = static_cast<int>(floorf(p_r.y));

  // 3. Scan through every corner to get the theta and calculate the average
  int2 corner;
  float2 dir=make_float2(0,0);
  float theta = 0;
  for (corner.x = lb_idx.x; corner.x<=lb_idx.x+1; corner.x++)
  {
    for (corner.y = lb_idx.y; corner.y<=lb_idx.y+1; corner.y++)
    {
#ifdef  __CUDA_ARCH__
      theta = map.theta_const_at(corner.x,corner.y,0);
#endif
      dir += calculate_weight(p_r,corner)*make_float2(cosf(theta),sinf(theta));
    }
  }

  return atan2f(dir.y,dir.x);
}
}
#endif // BILINEAR_INTERPOLATION_H
