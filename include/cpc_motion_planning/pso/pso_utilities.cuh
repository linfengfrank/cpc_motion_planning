#ifndef PSO_CUH
#define PSO_CUH

#include <curand_kernel.h>
#include <cuda_math/cuda_matrix.cuh>
namespace PSO
{
const int PSO_STEPS = 2; //planning steps
const float PSO_STEP_DT = 2; //lasting time of each step
const float PSO_TOTAL_T = PSO_STEPS*PSO_STEP_DT;

struct Trace
{
  float3 site[PSO_STEPS];

  __device__ __host__
  Trace()
  {
    for (unsigned int i=0;i<PSO_STEPS;i++)
    {
      site[i] = make_float3(0,0,0);
    }
  }

  __device__ __host__
  float3& operator[](unsigned int i)
  {
    return site[i];
  }

  __device__ __host__
  const float3& operator[](unsigned int i) const
  {
    return site[i];
  }

  __device__ __host__
  Trace operator-(Trace const &rhs) const
  {
    Trace tmp;
    for (unsigned int i=0;i<PSO_STEPS;i++)
    {
      tmp[i] = site[i] - rhs[i];
    }
    return tmp;
  }
  //---
  __device__ __host__
  Trace operator+(Trace const &rhs) const
  {
    Trace tmp;
    for (unsigned int i=0;i<PSO_STEPS;i++)
    {
      tmp[i] = site[i] + rhs[i];
    }
    return tmp;
  }
  //---
  __device__ __host__
  Trace operator*(float const &rhs) const
  {
    Trace tmp;
    for (unsigned int i=0;i<PSO_STEPS;i++)
    {
      tmp[i] = site[i] * rhs;
    }
    return tmp;
  }
  //---
  __device__ __host__
  Trace operator/(float const &rhs) const
  {
    Trace tmp;
    for (unsigned int i=0;i<PSO_STEPS;i++)
    {
      tmp[i] = site[i] / rhs;
    }
    return tmp;
  }
  //---
  __device__ __host__
  Trace& operator=(Trace const &rhs)
  {
    for (unsigned int i=0;i<PSO_STEPS;i++)
    {
      site[i] = rhs[i];
    }
    return *this;
  }
};

__host__ __device__ __forceinline__
float rand_float_gen(curandState *rs, float min, float max)
{
#ifdef  __CUDA_ARCH__
  float myrandf = curand_uniform(rs);
#else
  float myrandf = 0.0;
#endif
  myrandf *= (max - min + 0.999999f);
  myrandf += min;
  return myrandf;
}

__host__ __device__ __forceinline__
void bound_between(float &val, float min, float max)
{
    if (val < min)
        val = min;
    else if (val > max)
        val = max;
}

struct Particle
{
  Trace curr_loc;  //location
  Trace best_loc;  //self best
  Trace ptcl_vel;  //particle velocity
  float best_cost; //self best cost
  curandState rs;  //random state
};

struct Swarm
{
  Particle *ptcls;
  int ptcl_size;
};
}
#endif // PSO_CUH
