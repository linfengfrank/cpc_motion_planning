#ifndef PSO_CUH
#define PSO_CUH

#include <curand_kernel.h>
#include <cuda_math/cuda_matrix.cuh>
namespace PSO
{
const int PSO_STEPS = 1; //planning steps
const float PSO_STEP_DT = 4.0f; //lasting time of each step
const float PSO_TOTAL_T = PSO_STEPS*PSO_STEP_DT;
const float PSO_SIM_DT = 0.1f;
const float PSO_CTRL_DT = 0.05f;
const int PSO_REPLAN_CYCLE = 4;
const float PSO_REPLAN_DT = PSO_REPLAN_CYCLE * PSO_CTRL_DT;
const int PSO_PLAN_CONSUME_CYCLE = 2;
}
#endif // PSO_CUH
