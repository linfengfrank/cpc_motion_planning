#include <cpc_motion_planning/pso/pso_kernels.cuh>
#include <cuda_geometry/helper_math.h>
#include <cpc_motion_planning/pso/single_target_evluator.h>
namespace PSO
{
//---
template<class Model, class Evaluator>
__host__ __device__
float evaluate_trajectory(const Trace &tr, VoidPtrCarrier ptr_car,const UniformBinCarrier &ubc,
                          const EDTMap &map, const Trace &last_tr, const Evaluator &eva, Model &m)
{
  State s = m.get_ini_state();
  float cost = 0;
  float dt = PSO_SIM_DT;
  //float3 goal_p = goal.s.p;
  for (float t=0.0f; t<PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/PSO_STEP_DT));
    if (i > PSO_STEPS - 1)
      i = PSO_STEPS - 1;

    float3 u = dp_control(s, tr[i], ptr_car, ubc);
    m.model_forward(s,u,dt);

    cost += 0.1*sqrt(u.x*u.x + u.y*u.y + u.z*u.z);
    cost += eva.process_cost(s,map);

//    float3 ctr_pnt = tr[i];
//    float3 diff_tr = ctr_pnt - goal_p;

//    cost+= 0.05*sqrt(diff_tr.x*diff_tr.x + diff_tr.y*diff_tr.y + diff_tr.z*diff_tr.z);
  }
  cost += eva.final_cost(s,map);
//  Trace diff = tr - last_tr;
//  cost += sqrt(diff.square())/static_cast<float>(PSO_STEPS);
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
template<class Model, class Evaluator>
__global__
void initialize_particles_kernel(Swarm sw, bool first_run,
                                 VoidPtrCarrier ptr_car, UniformBinCarrier ubc,
                                 EDTMap map, Trace last_tr, Evaluator eva, Model m)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (first_run || idx != sw.ptcl_size-1)
  {
    initialize_a_particle(m.get_ini_state(), sw.ptcls[idx]);
  }
  sw.ptcls[idx].best_cost = evaluate_trajectory(sw.ptcls[idx].best_loc, ptr_car, ubc, map, last_tr,eva,m);

}

//---
template<class Model, class Evaluator>
__global__
void iterate_particles_kernel(Swarm sw, float weight,
                              VoidPtrCarrier ptr_car,  UniformBinCarrier ubc,
                              EDTMap map, Trace last_tr, Evaluator eva, Model m)
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
  bound_ptcl_location(sw.ptcls[idx], m.get_ini_state());

  float cost = evaluate_trajectory(sw.ptcls[idx].curr_loc, ptr_car, ubc, map, last_tr,eva,m);

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
template<class Model, class Evaluator>
void initialize_particles(const Swarm &sw, bool first_run,
                          VoidPtrCarrier ptr_car, const  UniformBinCarrier &ubc,
                          const EDTMap &map, const Trace &last_tr, const Evaluator &eva, const Model &m)
{
  initialize_particles_kernel<<<1,sw.ptcl_size>>>(sw,first_run,ptr_car,ubc, map, last_tr,eva,m);
}

//---------
template<class Model, class Evaluator>
void iterate_particles(const Swarm &sw, float weight,
                       VoidPtrCarrier ptr_car, const  UniformBinCarrier &ubc,
                       const EDTMap &map, const Trace &last_tr, const Evaluator &eva, const Model &m)
{
  iterate_particles_kernel<<<1,sw.ptcl_size>>>(sw,weight,ptr_car,ubc,map,last_tr,eva,m);
}

//---------
void copy_best_values(const Swarm &sw, float *best_values)
{
  copy_best_value_kernel<<<1,sw.ptcl_size>>>(sw.ptcls,best_values);
}

//float evaluate_trajectory_wrapper(const State &s0, const Trace &tr, VoidPtrCarrier ptr_car,const UniformBinCarrier &ubc,
//               const EDTMap &map, const Trace &last_tr)
//{
//  return 0;
////  return evaluate_trajectory(s0, goal, tr, ptr_car,ubc,
////                             map, last_tr);
//}

}

template void PSO::initialize_particles<PSO::UAVModel, PSO::SingleTargetEvaluator>(const Swarm &sw, bool first_run,
                                                                       VoidPtrCarrier ptr_car, const  UniformBinCarrier &ubc,
                                                                       const EDTMap &map, const Trace &last_tr, const PSO::SingleTargetEvaluator &eva, const PSO::UAVModel &m);


template void PSO::iterate_particles<PSO::UAVModel, PSO::SingleTargetEvaluator>(const Swarm &sw, float weight,
                                                                    VoidPtrCarrier ptr_car, const  UniformBinCarrier &ubc,
                                                                    const EDTMap &map, const Trace &last_tr, const PSO::SingleTargetEvaluator &eva, const PSO::UAVModel &m);

template float PSO::evaluate_trajectory<PSO::UAVModel, PSO::SingleTargetEvaluator>(const Trace &tr, VoidPtrCarrier ptr_car,const UniformBinCarrier &ubc,
                          const EDTMap &map, const Trace &last_tr, const PSO::SingleTargetEvaluator &eva, PSO::UAVModel &m);
