#ifndef PSO_CUH
#define PSO_CUH

#include <curand_kernel.h>
#include <cuda_math/cuda_matrix.cuh>
namespace PSO
{
const int PSO_STEPS = 2; //planning steps
const float PSO_STEP_DT = 2; //lasting time of each step
const float PSO_dt = 0.05f; //forward simulation dt
const float PSO_TOTAL_T = PSO_STEPS*PSO_STEP_DT;

struct Trace
{
  float3 site[PSO_STEPS];

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

struct Particle
{
  Trace curr_loc;  //location
  Trace best_loc;  //self best
  Trace ptcl_vel;  //particle velocity
  float best_cost; //self best cost
  curandState rs;  //random state
};
}
#endif // PSO_CUH
