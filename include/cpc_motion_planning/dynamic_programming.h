#ifndef DYNAMIC_PROGRAMMING_H
#define DYNAMIC_PROGRAMMING_H
#include <cuda_math/cuda_matrix.cuh>

struct dp_action
{
  float acc;
  float alpha;
};

namespace CUDA_MAT
{
// Only works for N = 1
__device__ __forceinline__
int search_idx(float val, const Vecf &bins)
{
  int M = static_cast<int>(bins.m_dim_width[0] - 1);
  int left = 0;
  int right = M;
  int mid = 0;
  int idx = 0;

  if (val >= bins.const_at(M))
  {
    idx = M-1;
  }
  else
  {
    while(1)
    {
      mid = (left+right)/2;
      if (val >= bins.const_at(mid))
      {
        left = mid;
      }
      else
      {
        right = mid;
      }

      if (right - left == 1)
      {
        idx = left;
        break;
      }
    }
  }
  return idx;
}
//---
__device__ __forceinline__
void get_opposite_pnt(int *loc, int *opp, const int *idx)
{
  for (int i=0;i<4;i++)
  {
    if (loc[i] == 0)
    {
      opp[i] = 1 + idx[i];
    }
    else
    {
      opp[i] = 0 + idx[i];
    }
  }
}
//---
__device__ __forceinline__
void bound(float &val, float min, float max)
{
  // bound the value
  if (val < min)
    val =  min;

  if(val > max)
    val =  max;
}
//---
__device__ __forceinline__
float get_value(float s[4], const CUDA_MAT::Mat4f &S, const Vecf &bins_0, const Vecf &bins_1, const Vecf &bins_2, const Vecf &bins_3)
{
  bound(s[0],bins_0.const_at(0), bins_0.const_at(static_cast<int>(bins_0.m_dim_width[0] - 1)));
  bound(s[1],bins_1.const_at(0), bins_1.const_at(static_cast<int>(bins_1.m_dim_width[0] - 1)));
  bound(s[2],bins_2.const_at(0), bins_2.const_at(static_cast<int>(bins_2.m_dim_width[0] - 1)));
  bound(s[3],bins_3.const_at(0), bins_3.const_at(static_cast<int>(bins_3.m_dim_width[0] - 1)));

  int idx[4];
  idx[0] = search_idx(s[0], bins_0);
  idx[1] = search_idx(s[1], bins_1);
  idx[2] = search_idx(s[2], bins_2);
  idx[3] = search_idx(s[3], bins_3);

  float volume = (bins_0.const_at(idx[0]+1)-bins_0.const_at(idx[0]))*
      (bins_1.const_at(idx[1]+1)-bins_1.const_at(idx[1]))*
      (bins_2.const_at(idx[2]+1)-bins_2.const_at(idx[2]))*
      (bins_3.const_at(idx[3]+1)-bins_3.const_at(idx[3]));

  int loc[4];
  int opp[4];
  float s_opp[4];
  float weight,val;
  float output = 0;
  for (loc[0] = 0;  loc[0]<= 1; loc[0]++)
  {
    for (loc[1] = 0;  loc[1]<= 1; loc[1]++)
    {
      for (loc[2] = 0;  loc[2]<= 1; loc[2]++)
      {
        for (loc[3] = 0;  loc[3]<= 1; loc[3]++)
        {
          get_opposite_pnt(loc,opp,idx);
          s_opp[0] = bins_0.const_at(opp[0]);
          s_opp[1] = bins_1.const_at(opp[1]);
          s_opp[2] = bins_2.const_at(opp[2]);
          s_opp[3] = bins_3.const_at(opp[3]);

          weight = fabs(s[0]-s_opp[0])*fabs(s[1]-s_opp[1])*fabs(s[2]-s_opp[2])*fabs(s[3]-s_opp[3])/volume;
          val = mat4f_get_val_const(loc[0]+idx[0],loc[1]+idx[1],loc[2]+idx[2],loc[3]+idx[3],S);
          output += weight * val;
        }
      }
    }
  }
  return output;
}
typedef Matrix<3, dp_action> Mat3Act;
typedef Matrix<4, dp_action> Mat4Act;
__device__ __forceinline__
dp_action& mat3act_get_val(int i, int j, int k, Mat3Act &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.at(idx);
}

__device__ __forceinline__
dp_action& mat4act_get_val(int i, int j, int k, int n, Mat4Act &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2]*mat.m_dim_width[3] + j*mat.m_dim_width[2]*mat.m_dim_width[3] + k*mat.m_dim_width[3] + n;
   return mat.at(idx);
}
}
namespace GPU_DP
{
__device__ __host__ __forceinline__
float pos_gen_val(int i)
{
//  if (i<15)
//  {
//    return -9.0f + 0.5f*static_cast<float>(i);
//  }
//  else if (i>34)
//  {
//    return -15.0f + 0.5f*static_cast<float>(i);
//  }
//  else
//  {
//    return -4.8f + 0.2f*static_cast<float>(i);
//  }
  return -6.0f + 0.2f*static_cast<float>(i);
}

__device__ __host__ __forceinline__
float vel_gen_val(int i)
{
   return -2.0f + 0.1f*static_cast<float>(i);
}

__device__ __host__ __forceinline__
float theta_gen_val(int i)
{
   return -3.14f + 0.13f*static_cast<float>(i);
}

__device__ __host__ __forceinline__
float w_gen_val(int i)
{
   return -2.5f + 0.1f*static_cast<float>(i);
}

void program(const CUDA_MAT::Mat4Act &S_A, const CUDA_MAT::Mat4f &S_1, const CUDA_MAT::Mat4f &S_2, const CUDA_MAT::Vecf &bin_p, const CUDA_MAT::Vecf &bin_v, const CUDA_MAT::Vecf &bin_theta
             ,const CUDA_MAT::Vecf &bin_w);
}
#endif
