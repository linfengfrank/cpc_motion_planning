#ifndef DYNAMIC_PROGRAMMING_H
#define DYNAMIC_PROGRAMMING_H
#include <cuda_math/cuda_matrix.cuh>

struct dp_action
{
  float jerk;
  __host__ __device__
  dp_action():jerk(0)
  {}
};

struct VoidPtrCarrier
{
  void * m_ptr[8];

  __host__ __device__
  void*& operator[](int i)
  {
    return m_ptr[i];
  }
};

struct UniformBin
{
  float min, max;
  float grid;
  int size;
};

struct UniformBinCarrier
{
  UniformBin bins[8];
};

namespace CUDA_MAT
{
typedef Matrix<3, dp_action> Mat3Act;
typedef Matrix<4, dp_action> Mat4Act;
__host__ __device__ __forceinline__
dp_action& mat3act_get_val(int i, int j, int k, Mat3Act &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.at(idx);
}

__host__ __device__ __forceinline__
const dp_action& mat3act_get_val_const(int i, int j, int k, const Mat3Act &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.const_at(idx);
}

__host__ __device__ __forceinline__
dp_action& mat4act_get_val(int i, int j, int k, int n, Mat4Act &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2]*mat.m_dim_width[3] + j*mat.m_dim_width[2]*mat.m_dim_width[3] + k*mat.m_dim_width[3] + n;
  return mat.at(idx);
}

__host__ __device__ __forceinline__
const dp_action& mat4act_get_val_const(int i, int j, int k, int n, const Mat4Act &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2]*mat.m_dim_width[3] + j*mat.m_dim_width[2]*mat.m_dim_width[3] + k*mat.m_dim_width[3] + n;
  return mat.const_at(idx);
}

// Only works for N = 1
__host__ __device__ __forceinline__
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
__host__ __device__ __forceinline__
void get_opposite_pnt(int *loc, int *opp, const int *idx, int size)
{
  for (int i=0;i<size;i++)
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
__host__ __device__ __forceinline__
void bound(float &val, float min, float max)
{
  // bound the value
  if (val < min)
    val =  min;

  if(val > max)
    val =  max;
}
//---
__host__ __device__ __forceinline__
float get_value_3(float s[3], const CUDA_MAT::Mat3f &S, const Vecf &bins_0, const Vecf &bins_1, const Vecf &bins_2)
{
  bound(s[0],bins_0.const_at(0), bins_0.const_at(static_cast<int>(bins_0.m_dim_width[0] - 1)));
  bound(s[1],bins_1.const_at(0), bins_1.const_at(static_cast<int>(bins_1.m_dim_width[0] - 1)));
  bound(s[2],bins_2.const_at(0), bins_2.const_at(static_cast<int>(bins_2.m_dim_width[0] - 1)));

  int idx[3];
  idx[0] = search_idx(s[0], bins_0);
  idx[1] = search_idx(s[1], bins_1);
  idx[2] = search_idx(s[2], bins_2);

  float volume = (bins_0.const_at(idx[0]+1)-bins_0.const_at(idx[0]))*
      (bins_1.const_at(idx[1]+1)-bins_1.const_at(idx[1]))*
      (bins_2.const_at(idx[2]+1)-bins_2.const_at(idx[2]));

  int loc[3];
  int opp[3];
  float s_opp[3];
  float weight,val;
  float output = 0;
  for (loc[0] = 0;  loc[0]<= 1; loc[0]++)
  {
    for (loc[1] = 0;  loc[1]<= 1; loc[1]++)
    {
      for (loc[2] = 0;  loc[2]<= 1; loc[2]++)
      {
        get_opposite_pnt(loc,opp,idx,3);
        s_opp[0] = bins_0.const_at(opp[0]);
        s_opp[1] = bins_1.const_at(opp[1]);
        s_opp[2] = bins_2.const_at(opp[2]);

        weight = fabs(s[0]-s_opp[0])*fabs(s[1]-s_opp[1])*fabs(s[2]-s_opp[2])/volume;
        val = mat3f_get_val_const(loc[0]+idx[0],loc[1]+idx[1],loc[2]+idx[2],S);
        output += weight * val;

      }
    }
  }
  return output;
}

__host__ __device__ __forceinline__
dp_action get_control_uniform_bin(float s[3], const CUDA_MAT::Mat3Act &SA, const UniformBinCarrier &ubc)
{
  int idx[3];
  float volume = 1.0f;
  for (int i=0; i<3; i++)
  {
    bound(s[i],ubc.bins[i].min, ubc.bins[i].max);
    idx[i] = floor((s[i] - ubc.bins[i].min) / ubc.bins[i].grid);

    if (idx[i] < 0)
      idx[i] = 0;

    if (idx[i] > ubc.bins[i].size -2)
      idx[i] = ubc.bins[i].size -2;

    volume *= ubc.bins[i].grid;
  }

  int loc[3];
  int opp[3];
  float s_opp[3];
  float weight;
  dp_action output,val;
  for (loc[0] = 0;  loc[0]<= 1; loc[0]++)
  {
    for (loc[1] = 0;  loc[1]<= 1; loc[1]++)
    {
      for (loc[2] = 0;  loc[2]<= 1; loc[2]++)
      {

          get_opposite_pnt(loc,opp,idx,3);

          for (int i=0; i<3; i++)
          {
            s_opp[i] = ubc.bins[i].min + ubc.bins[i].grid*static_cast<float>(opp[i]);
          }

          weight = fabs(s[0]-s_opp[0])*fabs(s[1]-s_opp[1])*fabs(s[2]-s_opp[2])*fabs(s[3]-s_opp[3])/volume;
          val = mat3act_get_val_const(loc[0]+idx[0],loc[1]+idx[1],loc[2]+idx[2],SA);
          output.jerk += weight * val.jerk;
      }
    }
  }
  return output;
}

}
namespace GPU_DP
{
__host__ __device__ __forceinline__
float pos_gen_val(int i)
{
  return -10.0f + 0.2f*static_cast<float>(i);
}

__host__ __device__ __forceinline__
float vel_gen_val(int i)
{
  return -5.0f + 0.2f*static_cast<float>(i);
}

__host__ __device__ __forceinline__
float acc_gen_val(int i)
{
  return -5.0f + 0.2f*static_cast<float>(i);
}

void program(VoidPtrCarrier ptr_car, size_t* bin_size);
}
#endif
