#include <cpc_motion_planning/pso/pso_kernels.cuh>
#include <cuda_geometry/helper_math.h>

namespace PSO
{
//---
template<class Swarm>
__global__
void setup_random_states_kernel(typename Swarm::Particle* tptcls)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  curand_init(9876, idx, 0, &(tptcls[idx].rs));
}

//---
template<class Model, class Controller, class Evaluator, class Swarm>
__global__
void initialize_particles_kernel(bool first_run,
                                 EDTMap map, Evaluator eva, Model m, Controller ctrl, Swarm sw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (first_run || idx != sw.ptcl_size-1)
  {
    sw.initialize_a_particle(m.get_ini_state(),sw.ptcls[idx]);
  }
  sw.ptcls[idx].best_cost = ctrl.template simulate_evaluate<Model,Evaluator,Swarm>(map,eva,m,sw,sw.ptcls[idx].best_loc);
}

//---
template<class Model, class Controller, class Evaluator, class Swarm>
__global__
void iterate_particles_kernel(float weight,
                              EDTMap map, Evaluator eva, Model m, Controller ctrl, Swarm sw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (idx == sw.ptcl_size-1)
    return;

  float r1 = PSO::rand_float_gen(&(sw.ptcls[idx].rs),0,1);
  float r2 = PSO::rand_float_gen(&(sw.ptcls[idx].rs),0,1);

  sw.ptcls[idx].ptcl_vel =
      sw.ptcls[idx].ptcl_vel*weight -
      (sw.ptcls[idx].curr_loc - sw.ptcls[idx].best_loc)*r1 -
      (sw.ptcls[idx].curr_loc - sw.ptcls[sw.ptcl_size-1].curr_loc)*r2;

  sw.bound_ptcl_velocity(sw.ptcls[idx]);

  sw.ptcls[idx].curr_loc = sw.ptcls[idx].curr_loc + sw.ptcls[idx].ptcl_vel;

  sw.bound_ptcl_location(m.get_ini_state(), sw.ptcls[idx]);

  float cost = ctrl.template simulate_evaluate<Model,Evaluator,Swarm>(map,eva,m,sw,sw.ptcls[idx].curr_loc);

  if (cost < sw.ptcls[idx].best_cost)
  {
    sw.ptcls[idx].best_cost = cost;
    sw.ptcls[idx].best_loc = sw.ptcls[idx].curr_loc;
  }
}

//---------
template<class Swarm>
__global__
void copy_best_value_kernel(float* best_values, Swarm sw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  best_values[idx] = sw.ptcls[idx].best_cost;
}

//---------
template<class Swarm>
void setup_random_states(const Swarm &sw)
{
  setup_random_states_kernel<Swarm><<<1,sw.ptcl_size>>>(sw.ptcls);
}

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
void initialize_particles(bool first_run,
                          const EDTMap &map,const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw)
{
  initialize_particles_kernel<Model,Controller,Evaluator,Swarm><<<1,sw.ptcl_size>>>(first_run,map,eva,m,ctrl,sw);
}

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
void iterate_particles(float weight,
                       const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw)
{
  iterate_particles_kernel<Model,Controller,Evaluator,Swarm><<<1,sw.ptcl_size>>>(weight,map,eva,m,ctrl,sw);
}

//---------
template<class Swarm>
void copy_best_values(float *best_values, const Swarm &sw)
{
  copy_best_value_kernel<Swarm><<<1,sw.ptcl_size>>>(best_values,sw);
}

//float evaluate_trajectory_wrapper(const UAVModel::State &s0, const Trace &tr, VoidPtrCarrier ptr_car,const UniformBinCarrier &ubc,
//               const EDTMap &map, const Trace &last_tr)
//{
//  return 0;
////  return evaluate_trajectory(s0, goal, tr, ptr_car,ubc,
////                             map, last_tr);
//}

}

#define INST_initialize_particles(M,C,E,S) template void PSO::initialize_particles< M,C,E,S > \
(bool, const EDTMap&, const E&, const M&, const C& , const S&);

#define INST_iterate_particles(M,C,E,S) template void PSO::iterate_particles< M,C,E,S > \
(float, const EDTMap&, const E&, const M&, const C& , const S&);

#define INST_setup_random_states(S) template void PSO::setup_random_states< S > \
(const S&);

#define INST_copy_best_values(S) template void PSO::copy_best_values< S > \
(float *best_values, const S&);

#define INST_group(M,C,E,S) INST_initialize_particles(M,C,E,S) \
  INST_iterate_particles(M,C,E,S) \
  INST_setup_random_states(S) \
  INST_copy_best_values(S)

INST_group(UAV::UAVModel, UAV::UAVDPControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1>);
INST_group(UAV::UAVModel, UAV::UAVDPControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<2>);

INST_initialize_particles(UAV::UAVModel, UAV::UAVJLTControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1>);
INST_iterate_particles(UAV::UAVModel, UAV::UAVJLTControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1>);

INST_initialize_particles(UAV::UAVModel, UAV::UAVJLTControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<2>);
INST_iterate_particles(UAV::UAVModel, UAV::UAVJLTControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<2>);

INST_initialize_particles(UAV::UAVModel, UAV::UAVDPControl, UAV::NF1Evaluator, UAV::UAVSwarm<1>);
INST_iterate_particles(UAV::UAVModel, UAV::UAVDPControl, UAV::NF1Evaluator, UAV::UAVSwarm<1>);

INST_initialize_particles(UAV::UAVModel, UAV::UAVJLTControl, UAV::NF1Evaluator, UAV::UAVSwarm<1>);
INST_iterate_particles(UAV::UAVModel, UAV::UAVJLTControl, UAV::NF1Evaluator, UAV::UAVSwarm<1>);

INST_initialize_particles(UAV::UAVModel, UAV::UAVDPControl, UAV::CorridorEvaluator, UAV::UAVSwarm<1>);
INST_iterate_particles(UAV::UAVModel, UAV::UAVDPControl, UAV::CorridorEvaluator, UAV::UAVSwarm<1>);
