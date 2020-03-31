#include <cpc_motion_planning/pso/pso_kernels.cuh>

namespace PSO
{
__global__
void setup_random_states_kernel(void *ptcls)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  curand_init(1234, idx, 0, &(static_cast<Particle*>(ptcls)[idx].rs));
  //    float myrandf = curand_uniform(&(static_cast<Particle*>(ptcls)[idx].rs));

  //    printf("%f\n",myrandf);
}

//---
template<int N>
__device__
void evaluate_trajectory(const State &s0, const Trace &tr, VoidPtrCarrier<N> ptr_car)
{
  State s = s0;
  for (float t=0.0f; t<PSO_TOTAL_T; t+=PSO_dt)
  {
    int i = static_cast<int>(floor(t/PSO_STEP_DT));
    if (i > PSO_STEPS - 1)
        i = PSO_STEPS - 1;

    float3 u = dp_control<N>(s, tr[i], ptr_car);
    model_forward(s,u,0.05);
    printf("%f %f\n",u.x,u.y);
  }
}
//---
template<int N>
__global__
void test_kernel(VoidPtrCarrier<N> ptr_car)
{
  State s;
  Trace tr;
  tr[0] = make_float3(2,2,0);
  tr[1] = make_float3(2,2,0);
  evaluate_trajectory<N>(s,tr,ptr_car);
}

//---
void setup_random_states(void *ptcls, int size)
{
  setup_random_states_kernel<<<1,size>>>(ptcls);
}

//---
template<int N>
void test_plan(VoidPtrCarrier<N> ptr_car)
{
  test_kernel<N><<<1,1>>>(ptr_car);
  cudaDeviceSynchronize();
}
}
template void PSO::test_plan<5>(VoidPtrCarrier<5> ptr_car);
