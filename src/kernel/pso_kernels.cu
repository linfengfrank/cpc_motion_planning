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
  float dt = 0.05f;
  for (float t=0.0f; t<PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/PSO_STEP_DT));
    if (i > PSO_STEPS - 1)
      i = PSO_STEPS - 1;

    float3 u = dp_control<N>(s, tr[i], ptr_car);
    model_forward(s,u,dt);
    cost += process_cost(s,goal);
  }
  cost += final_cost(s,goal);
  return cost;
}

//---
__global__
void setup_random_states_kernel(Particle *ptcls)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  curand_init(9876, idx, 0, &(ptcls[idx].rs));
}

//---
template <int N>
__global__
void initialize_particles_kernel(Particle *ptcls, int ptcl_size, bool first_run,
                                 State s0, State goal, VoidPtrCarrier<N> ptr_car)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  //if (first_run || idx != ptcl_size-1)
  //{
  initialize_a_particle(s0, ptcls[idx]);
  ptcls[idx].best_cost = evaluate_trajectory(s0, goal, ptcls[idx].best_loc, ptr_car);
  //}
}

//---
template <int N>
__global__
void iterate_particles_kernel(Particle *ptcls, int ptcl_size, float weight,
                              State s0, State goal, VoidPtrCarrier<N> ptr_car)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (idx == ptcl_size-1)
    return;

  float r1 = rand_float_gen(&(ptcls[idx].rs),0,1);
  float r2 = rand_float_gen(&(ptcls[idx].rs),0,1);

  ptcls[idx].ptcl_vel =
      ptcls[idx].ptcl_vel*weight -
      (ptcls[idx].curr_loc - ptcls[idx].best_loc)*r1 -
      (ptcls[idx].curr_loc - ptcls[ptcl_size-1].curr_loc)*r2;

  bound_ptcl_velocity(ptcls[idx]);

  ptcls[idx].curr_loc = ptcls[idx].curr_loc + ptcls[idx].ptcl_vel;
  bound_ptcl_location(ptcls[idx], s0);

  float cost = evaluate_trajectory(s0, goal, ptcls[idx].curr_loc, ptr_car);

  if (cost < ptcls[idx].best_cost)
  {
    ptcls[idx].best_cost = cost;
    ptcls[idx].best_loc = ptcls[idx].curr_loc;
  }
}

__global__
void copy_best_value_kernel(Particle *ptcls, float* best_values)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  best_values[idx] = ptcls[idx].best_cost;
  //printf("%f\n",best_values[idx]);
}

__global__
void copy_best_ptcl_kernel(Particle *ptcls, int ptcl_size, int best_idx)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  //ptcls[ptcl_size-1] = ptcls[best_idx];
  printf(">> %f, %f\n",ptcls[ptcl_size-1].best_cost, ptcls[best_idx].best_cost);
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
void setup_random_states(Particle *ptcls, int size)
{
  setup_random_states_kernel<<<1,size>>>(ptcls);
}

//---
template<int N>
void test_plan(const State &s, const State &goal, Particle *ptcls,
               float *best_values, int ptcls_size, Particle *result,
               bool first_run, VoidPtrCarrier<N> ptr_car,
               cublasHandle_t &_cbls_hdl)
{
//    State s;
//    State goal;
//    goal.p = make_float2(5,0);

  cublasStatus_t _cbls_stt;

  initialize_particles_kernel<N><<<1,ptcls_size>>>(ptcls,ptcls_size,first_run,s,goal,ptr_car);
  for (int i=0;i<40;i++)
  {
    iterate_particles_kernel<N><<<1,ptcls_size>>>(ptcls,ptcls_size,1.0f,s,goal,ptr_car);
    copy_best_value_kernel<<<1,ptcls_size>>>(ptcls,best_values);

    int best_idx = -1;
    _cbls_stt = cublasIsamin(_cbls_hdl,ptcls_size,best_values,1,&best_idx);

    if(best_idx != -1)
    {
      CUDA_MEMCPY_D2D(ptcls+ptcls_size-1,ptcls+best_idx-1,sizeof(Particle));
    }
  }

  CUDA_MEMCPY_D2H(result,ptcls+ptcls_size-1,sizeof(Particle));
  cudaDeviceSynchronize();

}
}
template void PSO::test_plan<5>(const State &s, const State &goal, Particle *ptcls,
float *best_values, int ptcls_size, Particle *result, bool first_run, VoidPtrCarrier<5> ptr_car, cublasHandle_t &_cbls_hdl);
