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

  __device__
  int grid2index(int g[N]) const
  {
    int idx = 0;
    for (int i = 0; i < N; ++i)
    {
      for (int j = i+1; j < N; ++j)
      {
        g[i] *= m_dim_width[j];
      }
      idx += g[i];
    }
    return idx;
  }

  __device__
  T& at(int g[N])
  {
    return m_data[grid2index(g)];
  }

  __device__
  const T& const_at(int g[N]) const
  {
    return m_data[grid2index(g)];
  }

  size_t m_dim_width[N];
  size_t m_byte_size;
  T *m_data;
};

typedef Matrix<1,float> Vecf;


// Only works for N = 1
__device__ __forceinline__
int search_idx(float val, const Vecf &bins)
{
    int M = static_cast<int>(bins.m_dim_width[0] - 1);
    int left = 0;
    int right = M;
    int mid = 0;
    int idx = 0;

    if (val >= bins.const_at(&M))
    {
        idx = M-1;
    }
    else
    {
        while(1)
        {
            mid = (left+right)/2;
            if (val >= bins.const_at(&mid))
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

}
#endif
