#ifndef CUDA_MATRIX_CUH
#define CUDA_MATRIX_CUH
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <cfloat>
#include <cassert>
#include <cuda_geometry/cuda_geometry.cuh>

namespace CUDA_MAT
{
template <int N, typename T>
class Matrix
{
public:
  Matrix(size_t dim_width[N])
  {
    m_byte_size = 1;
    for (int i=0; i<N; i++)
    {
      m_dim_width[i] = dim_width[i];
      m_byte_size *= m_dim_width[i];
    }
    m_byte_size *= sizeof(T);
  }

  void setup_device()
  {
    CUDA_ALLOC_DEV_MEM(&m_data,m_byte_size);
    CUDA_DEV_MEMSET(m_data,0,m_byte_size);
  }

  void free_device()
  {
    CUDA_FREE_DEV_MEM(m_data);
  }

  void upload_data(T* data)
  {
    CUDA_MEMCPY_H2D(m_data,data,m_byte_size);
  }

  void download_data(T* data)
  {
    CUDA_MEMCPY_D2H(data,m_data,m_byte_size);
  }

  ~Matrix()
  {

  }

//  __device__
//  int grid2index(int g[N]) const
//  {
//    int idx = 0;
//    for (int i = 0; i < N; ++i)
//    {
//      for (int j = i+1; j < N; ++j)
//      {
//        g[i] *= m_dim_width[j];
//      }
//      idx += g[i];
//    }
//    return idx;
//  }

//  __device__
//  T& at(int g[N])
//  {
//    return m_data[grid2index(g)];
//  }

//  __device__
//  const T& const_at(int g[N]) const
//  {
//    return m_data[grid2index(g)];
//  }

  __device__
  T& at(int i)
  {
    return m_data[i];
  }

  __device__
  const T& const_at(int i) const
  {
    return m_data[i];
  }

  size_t m_dim_width[N];
  size_t m_byte_size;
  T *m_data;
};

typedef Matrix<1,float> Vecf;
typedef Matrix<3,float> Mat3f;
typedef Matrix<4,float> Mat4f;

__device__ __forceinline__
float& mat3f_get_val(int i, int j, int k, Mat3f &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.at(idx);
}

__device__ __forceinline__
const float& mat3f_get_val_const(int i, int j, int k, const Mat3f &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.const_at(idx);
}

__device__ __forceinline__
float& mat4f_get_val(int i, int j, int k, int n, Mat4f &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2]*mat.m_dim_width[3] + j*mat.m_dim_width[2]*mat.m_dim_width[3] + k*mat.m_dim_width[3] + n;
  return mat.at(idx);
}

__device__ __forceinline__
const float& mat4f_get_val_const(int i, int j, int k, int n, const Mat4f &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2]*mat.m_dim_width[3] + j*mat.m_dim_width[2]*mat.m_dim_width[3] + k*mat.m_dim_width[3] + n;
  return mat.const_at(idx);
}
}
#endif
