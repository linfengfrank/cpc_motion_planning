#include <cpc_motion_planning/pso/pso_kernels.cuh>

namespace PSO
{
//---
template<int N>
__host__ __device__
float evaluate_trajectory(const State &s0, const State &goal, const Trace &tr, VoidPtrCarrier<N> ptr_car)
{
  State s = s0;
  float cost = 0;
  for (float t=0.0f; t<PSO_TOTAL_T; t+=PSO_dt)
  {
    int i = static_cast<int>(floor(t/PSO_STEP_DT));
    if (i > PSO_STEPS - 1)
        i = PSO_STEPS - 1;

    float3 u = dp_control<N>(s, tr[i], ptr_car);
    model_forward(s,u,0.05);
    cost += process_cost(s,goal);
  }
  cost += final_cost(s,goal);
  return cost;
}

//---
__global__
void setup_random_states_kernel(void *ptcls)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  curand_init(1234, idx, 0, &(static_cast<Mat1P*>(ptcls)->at(idx).rs));
}

//---
template <int N>
__global__
void initialize_particles_kernel(void *ptcls, int ptcl_size, bool first_run,
                                 State s0, State goal, VoidPtrCarrier<N> ptr_car)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  Mat1P* particles = static_cast<Mat1P*>(ptcls);

  if (first_run || idx != ptcl_size-1)
  {
    initialize_a_particle(s0, particles->at(idx));
    particles->at(idx).best_cost = evaluate_trajectory(s0, goal, particles->at(idx).best_loc, ptr_car);
  }
}

//---
template<int N>
__global__
void test_kernel(VoidPtrCarrier<N> ptr_car)
{
  State s;
  Trace tr;
  State goal;
  goal.p = make_float2(10,10);
  tr[0] = make_float3(2,2,0);
  tr[1] = make_float3(2,2,0);
  float cost = evaluate_trajectory<N>(s,goal,tr,ptr_car);
  printf("Cost: %f\n",cost);
}

//---
void setup_random_states(void *ptcls, int size)
{
  setup_random_states_kernel<<<1,size>>>(ptcls);
}

//---
template<int N>
void test_plan(void *ptcls, int ptcls_size, bool first_run, VoidPtrCarrier<N> ptr_car)
{
  //test_kernel<N><<<1,1>>>(ptr_car);
  State s;
  State goal;
  goal.p = make_float2(10,10);
  initialize_particles_kernel<N><<<1,2>>>(ptcls,ptcls_size,first_run,s,goal,ptr_car);
  cudaDeviceSynchronize();
}
}
template void PSO::test_plan<5>(void *ptcls, int ptcls_size, bool first_run, VoidPtrCarrier<5> ptr_car);
