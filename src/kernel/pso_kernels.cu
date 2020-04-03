#include <cpc_motion_planning/pso/pso_kernels.cuh>

namespace PSO
{
//---
template<int N>
__host__ __device__
float evaluate_trajectory(const State &s0, const State &goal, const Trace &tr, VoidPtrCarrier<N> ptr_car,const UniformBinCarrier &ubc,
                          const EDTMap &map, const Trace &last_tr)
{
  State s = s0;
  float cost = 0;
  float dt = 0.1f;
  for (float t=0.0f; t<PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/PSO_STEP_DT));
    if (i > PSO_STEPS - 1)
      i = PSO_STEPS - 1;

    float3 u = dp_control<N>(s, tr[i], ptr_car, ubc);
    model_forward(s,u,dt);
    cost += process_cost(s,goal,map);
  }
  cost += final_cost(s,goal,map);
  Trace diff = tr - last_tr;
  cost += sqrt(diff.square());
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
void initialize_particles_kernel(Swarm sw, bool first_run,
                                 State s0, State goal, VoidPtrCarrier<N> ptr_car, UniformBinCarrier ubc,
                                 EDTMap map, Trace last_tr)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (first_run || idx != sw.ptcl_size-1)
  {
    initialize_a_particle(s0, sw.ptcls[idx]);
  }
  sw.ptcls[idx].best_cost = evaluate_trajectory(s0, goal, sw.ptcls[idx].best_loc, ptr_car, ubc, map, last_tr);

}

//---
template <int N>
__global__
void iterate_particles_kernel(Swarm sw, float weight,
                              State s0, State goal, VoidPtrCarrier<N> ptr_car,  UniformBinCarrier ubc,
                              EDTMap map, Trace last_tr)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (idx == sw.ptcl_size-1)
    return;

  float r1 = rand_float_gen(&(sw.ptcls[idx].rs),0,1);
  float r2 = rand_float_gen(&(sw.ptcls[idx].rs),0,1);

  sw.ptcls[idx].ptcl_vel =
      sw.ptcls[idx].ptcl_vel*weight -
      (sw.ptcls[idx].curr_loc - sw.ptcls[idx].best_loc)*r1 -
      (sw.ptcls[idx].curr_loc - sw.ptcls[sw.ptcl_size-1].curr_loc)*r2;

  bound_ptcl_velocity(sw.ptcls[idx]);

  sw.ptcls[idx].curr_loc = sw.ptcls[idx].curr_loc + sw.ptcls[idx].ptcl_vel;
  bound_ptcl_location(sw.ptcls[idx], s0);

  float cost = evaluate_trajectory(s0, goal, sw.ptcls[idx].curr_loc, ptr_car, ubc, map, last_tr);

  if (cost < sw.ptcls[idx].best_cost)
  {
    sw.ptcls[idx].best_cost = cost;
    sw.ptcls[idx].best_loc = sw.ptcls[idx].curr_loc;
  }
}

//---------
__global__
void copy_best_value_kernel(Particle *ptcls, float* best_values)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  best_values[idx] = ptcls[idx].best_cost;
}

//---------
void setup_random_states(const Swarm &sw)
{
  setup_random_states_kernel<<<1,sw.ptcl_size>>>(sw.ptcls);
}

//---------
template<int N>
void initialize_particles(const Swarm &sw, bool first_run,
                          const State &s, const State &goal,VoidPtrCarrier<N> ptr_car, const  UniformBinCarrier &ubc,
                          const EDTMap &map, const Trace &last_tr)
{
  initialize_particles_kernel<N><<<1,sw.ptcl_size>>>(sw,first_run,s,goal,ptr_car,ubc, map, last_tr);
}

//---------
template<int N>
void iterate_particles(const Swarm &sw, float weight,
                       const State &s, const State &goal,VoidPtrCarrier<N> ptr_car, const  UniformBinCarrier &ubc,
                       const EDTMap &map, const Trace &last_tr)
{
  iterate_particles_kernel<N><<<1,sw.ptcl_size>>>(sw,weight,s,goal,ptr_car,ubc,map,last_tr);
}

//---------
void copy_best_values(const Swarm &sw, float *best_values)
{
  copy_best_value_kernel<<<1,sw.ptcl_size>>>(sw.ptcls,best_values);
}
}

template void PSO::initialize_particles<5>(const Swarm &sw, bool first_run,
const State &s, const State &goal,VoidPtrCarrier<5> ptr_car, const  UniformBinCarrier &ubc,
const EDTMap &map, const Trace &last_tr);

template void PSO::iterate_particles<5>(const Swarm &sw, float weight,
const State &s, const State &goal,VoidPtrCarrier<5> ptr_car, const  UniformBinCarrier &ubc,
const EDTMap &map, const Trace &last_tr);
