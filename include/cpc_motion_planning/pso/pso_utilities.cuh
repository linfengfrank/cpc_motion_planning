#ifndef PSO_CUH
#define PSO_CUH

#include <curand_kernel.h>
#include <cuda_math/cuda_matrix.cuh>
namespace PSO
{
//const int PSO_STEPS = 1; //planning steps
//const float PSO_STEP_DT = 4.0f; //lasting time of each step
//const float PSO_TOTAL_T = 4.0f;
const float PSO_SIM_DT = 0.1f;
const float PSO_CTRL_DT = 0.05f;
const float MIN_DIST = 0.351f;
const int PSO_REPLAN_CYCLE = 4;
const float PSO_REPLAN_DT = PSO_REPLAN_CYCLE * PSO_CTRL_DT;
const int PSO_PLAN_CONSUME_CYCLE = 2;

enum CollisionState
{
  FREE = 0,
  COLLISION,
  INIT_COLLISION
};

struct EvaData
{
  bool collision;
  float min_dist;
  __host__ __device__
  EvaData():collision(false),min_dist(0)
  {

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
  myrandf *= (max - min);
  myrandf += min;
  return myrandf;
}

__host__ __device__ __forceinline__
int rand_int_gen(curandState *rs, int min, int max)
{
  float output_float = rand_float_gen(rs, static_cast<float>(min), static_cast<float>(max)+0.9999f);
  int output = static_cast<int>(output_float);

  if (output > max) output = max;
  if (output < min) output = min;

  return output;
}

__host__ __device__ __forceinline__
void bound_between(float &val, float min, float max)
{
    if (val < min)
        val = min;
    else if (val > max)
        val = max;
}

__host__ __device__ __forceinline__
void modulo_bound_between(float &val, float min, float max)
{
  float m = max - min;
  val = val - floorf((val-min)/m)*m;
}
}
#endif // PSO_CUH
