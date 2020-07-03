#ifndef DYNAMIC_PROGRAMMING_H
#define DYNAMIC_PROGRAMMING_H
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/uav/model/uav_model.h>

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
template<typename T>
__host__ __device__ __forceinline__
T& mat2_get_val(int i, int j, Matrix<2, T> &mat)
{
  int idx = i*mat.m_dim_width[1] + j;
  return mat.at(idx);
}

template<typename T>
__host__ __device__ __forceinline__
const T& mat2_get_val_const(int i, int j, const Matrix<2, T> &mat)
{
  int idx = i*mat.m_dim_width[1] + j;
  return mat.const_at(idx);
}

template<typename T>
__host__ __device__ __forceinline__
T& mat3_get_val(int i, int j, int k,  Matrix<3, T> &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.at(idx);
}

template<typename T>
__host__ __device__ __forceinline__
const T& mat3_get_val_const(int i, int j, int k, const Matrix<3, T> &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.const_at(idx);
}

template<typename T>
__host__ __device__ __forceinline__
T& mat4_get_val(int i, int j, int k, int n, Matrix<4,T> &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2]*mat.m_dim_width[3] + j*mat.m_dim_width[2]*mat.m_dim_width[3] + k*mat.m_dim_width[3] + n;
  return mat.at(idx);
}

template<typename T>
__host__ __device__ __forceinline__
const T& mat4_get_val_const(int i, int j, int k, int n, const Matrix<4,T> &mat)
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
float get_value_2(float s[2], const Matrix<2,float> &S, const Vecf &bins_0, const Vecf &bins_1)
{
  bound(s[0],bins_0.const_at(0), bins_0.const_at(static_cast<int>(bins_0.m_dim_width[0] - 1)));
  bound(s[1],bins_1.const_at(0), bins_1.const_at(static_cast<int>(bins_1.m_dim_width[0] - 1)));

  int idx[2];
  idx[0] = search_idx(s[0], bins_0);
  idx[1] = search_idx(s[1], bins_1);

  float volume = (bins_0.const_at(idx[0]+1)-bins_0.const_at(idx[0]))*
      (bins_1.const_at(idx[1]+1)-bins_1.const_at(idx[1]));

  int loc[2];
  int opp[2];
  float s_opp[2];
  float weight,val;
  float output = 0;
  for (loc[0] = 0;  loc[0]<= 1; loc[0]++)
  {
    for (loc[1] = 0;  loc[1]<= 1; loc[1]++)
    {
      get_opposite_pnt(loc,opp,idx,2);
      s_opp[0] = bins_0.const_at(opp[0]);
      s_opp[1] = bins_1.const_at(opp[1]);

      weight = fabsf(s[0]-s_opp[0])*fabsf(s[1]-s_opp[1])/volume;
      val = mat2_get_val_const<float>(loc[0]+idx[0],loc[1]+idx[1],S);
      output += weight * val;

    }
  }
  return output;
}
//---
__host__ __device__ __forceinline__
float get_value_3(float s[3], const Matrix<3,float> &S, const Vecf &bins_0, const Vecf &bins_1, const Vecf &bins_2)
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

        weight = fabsf(s[0]-s_opp[0])*fabsf(s[1]-s_opp[1])*fabsf(s[2]-s_opp[2])/volume;
        val = mat3_get_val_const<float>(loc[0]+idx[0],loc[1]+idx[1],loc[2]+idx[2],S);
        output += weight * val;

      }
    }
  }
  return output;
}

//---
__host__ __device__ __forceinline__
float get_value_4(float s[4], const Matrix<4,float> &S, const Vecf &bins_0, const Vecf &bins_1, const Vecf &bins_2, const Vecf &bins_3)
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
          get_opposite_pnt(loc,opp,idx,4);
          s_opp[0] = bins_0.const_at(opp[0]);
          s_opp[1] = bins_1.const_at(opp[1]);
          s_opp[2] = bins_2.const_at(opp[2]);
          s_opp[3] = bins_3.const_at(opp[3]);

          weight = fabsf(s[0]-s_opp[0])*fabsf(s[1]-s_opp[1])*fabsf(s[2]-s_opp[2])*fabsf(s[3]-s_opp[3])/volume;
          val = mat4_get_val_const<float>(loc[0]+idx[0],loc[1]+idx[1],loc[2]+idx[2],loc[3]+idx[3],S);
          output += weight * val;
        }
      }
    }
  }
  return output;
}
//---
template<typename action>
__host__ __device__ __forceinline__
action get_control_uniform_bin_4(float s[4], const Matrix<4,action> &SA, const UniformBinCarrier &ubc)
{
  int idx[4];
  float volume = 1.0f;
  for (int i=0; i<4; i++)
  {
    bound(s[i],ubc.bins[i].min, ubc.bins[i].max);
    idx[i] = floorf((s[i] - ubc.bins[i].min) / ubc.bins[i].grid);

    if (idx[i] < 0)
      idx[i] = 0;

    if (idx[i] > ubc.bins[i].size -2)
      idx[i] = ubc.bins[i].size -2;

    volume *= ubc.bins[i].grid;
  }

  int loc[4];
  int opp[4];
  float s_opp[4];
  float weight;
  action output,val;
  for (loc[0] = 0;  loc[0]<= 1; loc[0]++)
  {
    for (loc[1] = 0;  loc[1]<= 1; loc[1]++)
    {
      for (loc[2] = 0;  loc[2]<= 1; loc[2]++)
      {
        for (loc[3] = 0;  loc[3]<= 1; loc[3]++)
        {
          get_opposite_pnt(loc,opp,idx,4);

          for (int i=0; i<4; i++)
          {
            s_opp[i] = ubc.bins[i].min + ubc.bins[i].grid*static_cast<float>(opp[i]);
          }

          weight = fabsf(s[0]-s_opp[0])*fabsf(s[1]-s_opp[1])*fabsf(s[2]-s_opp[2])*fabsf(s[3]-s_opp[3])/volume;
          val = mat4_get_val_const<action>(loc[0]+idx[0],loc[1]+idx[1],loc[2]+idx[2],loc[3]+idx[3],SA);
          output.acc += weight * val.acc;
          output.alpha += weight * val.alpha;
        }
      }
    }
  }
  return output;
}
//---
template<typename action>
__host__ __device__ __forceinline__
action get_control_uniform_bin_3(float s[3], const Matrix<3,action> &SA, const UniformBinCarrier &ubc)
{
  int idx[3];
  float volume = 1.0f;
  for (int i=0; i<3; i++)
  {
    bound(s[i],ubc.bins[i].min, ubc.bins[i].max);
    idx[i] = floorf((s[i] - ubc.bins[i].min) / ubc.bins[i].grid);

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
  action output,val;
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

        weight = fabsf(s[0]-s_opp[0])*fabsf(s[1]-s_opp[1])*fabsf(s[2]-s_opp[2])/volume;
        val = mat3_get_val_const<action>(loc[0]+idx[0],loc[1]+idx[1],loc[2]+idx[2],SA);
        output.jerk += weight * val.jerk;
      }
    }
  }
  return output;
}
//---
template<typename action>
__host__ __device__ __forceinline__
action get_control_uniform_bin_2(float s[2], const Matrix<2,action> &SA, const UniformBinCarrier &ubc)
{
  int idx[2];
  float volume = 1.0f;
  for (int i=0; i<2; i++)
  {
    bound(s[i],ubc.bins[i].min, ubc.bins[i].max);
    idx[i] = floorf((s[i] - ubc.bins[i].min) / ubc.bins[i].grid);

    if (idx[i] < 0)
      idx[i] = 0;

    if (idx[i] > ubc.bins[i].size -2)
      idx[i] = ubc.bins[i].size -2;

    volume *= ubc.bins[i].grid;
  }

  int loc[2];
  int opp[2];
  float s_opp[2];
  float weight;
  action output,val;
  for (loc[0] = 0;  loc[0]<= 1; loc[0]++)
  {
    for (loc[1] = 0;  loc[1]<= 1; loc[1]++)
    {

        get_opposite_pnt(loc,opp,idx,2);

        for (int i=0; i<2; i++)
        {
          s_opp[i] = ubc.bins[i].min + ubc.bins[i].grid*static_cast<float>(opp[i]);
        }

        weight = fabsf(s[0]-s_opp[0])*fabsf(s[1]-s_opp[1])/volume;
        val = mat2_get_val_const<action>(loc[0]+idx[0],loc[1]+idx[1],SA);
        output.jerk += weight * val.jerk;

    }
  }
  return output;
}


}

namespace UAV
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
}

namespace GPU_DP
{
template<typename action>
void program(VoidPtrCarrier ptr_car, size_t* bin_size);

template<typename action>
void program_vel(VoidPtrCarrier ptr_car, size_t* bin_size);
}
#endif
