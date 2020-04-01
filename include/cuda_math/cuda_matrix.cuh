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
#include <fstream>

namespace CUDA_MAT
{
template <int N, typename T>
class Matrix
{
public:
  Matrix(size_t dim_width[N], bool on_host = false):
    m_on_host(on_host)
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
    if (!m_on_host)
    {
      CUDA_ALLOC_DEV_MEM(&m_data,m_byte_size);
      CUDA_DEV_MEMSET(m_data,0,m_byte_size);
    }
    else
    {
      m_data = static_cast<T*>(malloc(m_byte_size));
      memset(m_data, 0, m_byte_size);
    }
  }

  void free_device()
  {
    if (!m_on_host)
    {
      CUDA_FREE_DEV_MEM(m_data);
    }
    else
    {
      free(m_data);
    }
  }

  void upload_data(T* data)
  {
    if (!m_on_host)
    {
      CUDA_MEMCPY_H2D(m_data,data,m_byte_size);
    }
    else
    {
      memcpy(m_data,data,m_byte_size);
    }
  }

  void download_data(T* data)
  {
    if (!m_on_host)
    {
      CUDA_MEMCPY_D2H(data,m_data,m_byte_size);
    }
    else
    {
      memcpy(data,m_data,m_byte_size);
    }
  }

  void write_to_file(std::string file_name)
  {
    // open the file
    std::ofstream file;
    file.open(file_name.c_str(), std::ios::binary);

    // construct a small header to record the dimension of this hyper matrix
    int* header = new int[1 + N];
    header[0] = N;
    int size = 1;
    for (int i=0; i<N; i++)
    {
      header[i+1] = static_cast<int>(m_dim_width[i]);
      size *= m_dim_width[i];
    }
    file.write(reinterpret_cast<const char *>(header),sizeof(int)*(N+1));
    delete [] header;

    // download data from device and write it to file
    T* data = new T[size];
    download_data(data);
    file.write(reinterpret_cast<const char *>(data), m_byte_size);
    delete [] data;

    // close the file
    file.close();
  }

  ~Matrix()
  {

  }

  __host__ __device__
  T& at(int i)
  {
    return m_data[i];
  }

  __host__ __device__
  const T& const_at(int i) const
  {
    return m_data[i];
  }

  size_t m_dim_width[N];
  bool m_on_host;
  size_t m_byte_size;
  T *m_data;
};

typedef Matrix<1,float> Vecf;
typedef Matrix<3,float> Mat3f;
typedef Matrix<4,float> Mat4f;

__host__ __device__ __forceinline__
float& mat3f_get_val(int i, int j, int k, Mat3f &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.at(idx);
}

__host__ __device__ __forceinline__
const float& mat3f_get_val_const(int i, int j, int k, const Mat3f &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2] + j*mat.m_dim_width[2] + k;
  return mat.const_at(idx);
}

__host__ __device__ __forceinline__
float& mat4f_get_val(int i, int j, int k, int n, Mat4f &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2]*mat.m_dim_width[3] + j*mat.m_dim_width[2]*mat.m_dim_width[3] + k*mat.m_dim_width[3] + n;
  return mat.at(idx);
}

__host__ __device__ __forceinline__
const float& mat4f_get_val_const(int i, int j, int k, int n, const Mat4f &mat)
{
  int idx = i*mat.m_dim_width[1]*mat.m_dim_width[2]*mat.m_dim_width[3] + j*mat.m_dim_width[2]*mat.m_dim_width[3] + k*mat.m_dim_width[3] + n;
  return mat.const_at(idx);
}
}
#endif
