#ifndef PSO_CUH
#define PSO_CUH

#include <curand_kernel.h>
#include <cuda_math/cuda_matrix.cuh>
namespace PSO
{
//const int PSO_STEPS = 1; //planning steps
//const float PSO_STEP_DT = 4.0f; //lasting time of each step
const float PSO_TOTAL_T = 4.0f;
const float PSO_SIM_DT = 0.1f;
const float PSO_CTRL_DT = 0.05f;
const int PSO_REPLAN_CYCLE = 4;
const float PSO_REPLAN_DT = PSO_REPLAN_CYCLE * PSO_CTRL_DT;
const int PSO_PLAN_CONSUME_CYCLE = 2;

__host__ __device__ __forceinline__
float rand_float_gen(curandState *rs, float min, float max)
{
#ifdef  __CUDA_ARCH__
  float myrandf = curand_uniform(rs);
#else
  float myrandf = 0.0;
#endif
  myrandf *= (max - min);
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
}
#endif // PSO_CUH
