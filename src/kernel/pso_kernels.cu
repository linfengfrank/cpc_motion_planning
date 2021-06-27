#include <cpc_motion_planning/pso/pso_kernels.cuh>
#include <cuda_geometry/helper_math.h>

namespace PSO
{
//---
template<class Model, class Swarm>
__device__ __forceinline__
void de_cross_over_particle(const Model &m, Swarm &sw, int idx)
{
  // pick three distinct particles from the swarm (r1,r2 and r3):
  int r1 = PSO::rand_int_gen(&(sw.ptcls[idx].rs),0,sw.ptcl_size-1);
  while (r1 == idx)
    r1 = PSO::rand_int_gen(&(sw.ptcls[idx].rs),0,sw.ptcl_size-1);

  int r2 = PSO::rand_int_gen(&(sw.ptcls[idx].rs),0,sw.ptcl_size-1);
  while (r2 == idx || r2 == r1)
    r2 = PSO::rand_int_gen(&(sw.ptcls[idx].rs),0,sw.ptcl_size-1);

  int r3 = PSO::rand_int_gen(&(sw.ptcls[idx].rs),0,sw.ptcl_size-1);
  while (r3 == idx || r3 == r1 || r3 == r2)
    r3 = PSO::rand_int_gen(&(sw.ptcls[idx].rs),0,sw.ptcl_size-1);

  // construct the variation
  typename Swarm::Trace var = sw.ptcls[r1].best_loc + (sw.ptcls[r2].best_loc - sw.ptcls[r3].best_loc)*0.8f;

  // random sample an index r, to be used in cross over later
  int r = PSO::rand_int_gen(&(sw.ptcls[idx].rs),0,sw.steps*2-1);

  // corssover
  float cr;
  int step, dim;
  for (int i=0; i<sw.steps*2; i++)
  {
    cr = PSO::rand_float_gen(&(sw.ptcls[idx].rs),0,1);
    step = i/2;
    dim = i%2;
    if (cr < 0.9f || i == r)
    {
      if (dim == 0)
        sw.ptcls[idx].curr_loc[step].x = var[step].x;
      else
        sw.ptcls[idx].curr_loc[step].y = var[step].y;
    }
    else
    {
      if (dim == 0)
        sw.ptcls[idx].curr_loc[step].x = sw.ptcls[idx].best_loc[step].x;
      else
        sw.ptcls[idx].curr_loc[step].y = sw.ptcls[idx].best_loc[step].y;
    }
  }

  //bound the particle
  sw.bound_ptcl_location(m.get_ini_state(), sw.ptcls[idx]);
}
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
  // Initialize the particles curr_loc, ptcl_vel and best_loc
  sw.initialize_a_particle(m.get_ini_state(),sw.ptcls[idx]);

  // Initialize the glb_best only at first run together with the first particle
  if (idx == 0)
  {
    if(first_run)
    {
      sw.initialize_a_particle(m.get_ini_state(),*(sw.best_ptcl));
    }

    sw.bound_ptcl_location(m.get_ini_state(), *(sw.best_ptcl));
    sw.bound_ptcl_best_location(m.get_ini_state(), *(sw.best_ptcl));

  }

  // Evaluate the initialized particle
  sw.ptcls[idx].best_cost = ctrl.template simulate_evaluate<Model,Evaluator,Swarm >(map,eva,m, sw, sw.ptcls[idx].best_loc, sw.ptcls[idx].collision);

  // Evaluate the best particle together with the first one
  if (idx == 0)
    sw.best_ptcl->best_cost = ctrl.template simulate_evaluate<Model,Evaluator,Swarm >(map,eva,m, sw, sw.best_ptcl->best_loc, sw.best_ptcl->collision);
}

//---
template<class Model, class Controller, class Evaluator, class Swarm>
__global__
void iterate_particles_kernel(float2 weight,
                              EDTMap map, Evaluator eva, Model m, Controller ctrl, Swarm sw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  float r1 = PSO::rand_float_gen(&(sw.ptcls[idx].rs),0,1);
  float r2 = PSO::rand_float_gen(&(sw.ptcls[idx].rs),0,1);

  sw.ptcls[idx].ptcl_vel =
      sw.ptcls[idx].ptcl_vel*weight.x -
      (sw.ptcls[idx].curr_loc - sw.ptcls[idx].best_loc)*r1 -
      (sw.ptcls[idx].curr_loc - sw.best_ptcl->best_loc)*r2;

  sw.bound_ptcl_velocity(sw.ptcls[idx]);

  sw.ptcls[idx].curr_loc = sw.ptcls[idx].curr_loc + sw.ptcls[idx].ptcl_vel;

  sw.bound_ptcl_location(m.get_ini_state(), sw.ptcls[idx]);


  float rr = PSO::rand_float_gen(&(sw.ptcls[idx].rs),0,1);

  if (rr < weight.y)
  {
    de_cross_over_particle<Model, Swarm>(m,sw,idx);
  }

  float cost = ctrl.template simulate_evaluate<Model,Evaluator,Swarm >(map,eva,m,sw,sw.ptcls[idx].curr_loc,sw.ptcls[idx].collision);

  __syncthreads();

  if (cost < sw.ptcls[idx].best_cost && !sw.ptcls[idx].collision)
  {
    sw.ptcls[idx].best_cost = cost;
    sw.ptcls[idx].best_loc = sw.ptcls[idx].curr_loc;
  }
}
//---------
//---
template<class Model, class Controller, class Evaluator, class Swarm>
__global__
void iterate_particles_de_kernel(EDTMap map, Evaluator eva, Model m, Controller ctrl, Swarm sw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  de_cross_over_particle<Model, Swarm>(m,sw,idx);

  //calculate the cost
  float cost = ctrl.template simulate_evaluate<Model,Evaluator,Swarm >(map,eva,m,sw,sw.ptcls[idx].curr_loc,sw.ptcls[idx].collision);

  __syncthreads();

  //update the cost
  if (cost < sw.ptcls[idx].best_cost && !sw.ptcls[idx].collision)
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
__global__
void update_glb_best_kernel(int sw_bst_id, Swarm sw)
{
  // Do not copy the random generator
  if (sw.ptcls[sw_bst_id].best_cost < sw.best_ptcl->best_cost)
  {
    sw.best_ptcl->curr_loc = sw.ptcls[sw_bst_id].curr_loc;
    sw.best_ptcl->best_loc = sw.ptcls[sw_bst_id].best_loc;
    sw.best_ptcl->ptcl_vel = sw.ptcls[sw_bst_id].ptcl_vel;
    sw.best_ptcl->best_cost = sw.ptcls[sw_bst_id].best_cost;
    sw.best_ptcl->collision = sw.ptcls[sw_bst_id].collision;
  }
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
void iterate_particles(float2 weight,
                       const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw)
{
  iterate_particles_kernel<Model,Controller,Evaluator,Swarm><<<1,sw.ptcl_size>>>(weight,map,eva,m,ctrl,sw);
}

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
void iterate_particles_de(const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw)
{
  iterate_particles_de_kernel<Model,Controller,Evaluator,Swarm><<<1,sw.ptcl_size>>>(map,eva,m,ctrl,sw);
}

//---------
template<class Swarm>
void copy_best_values(float *best_values, const Swarm &sw)
{
  copy_best_value_kernel<Swarm><<<1,sw.ptcl_size>>>(best_values,sw);
}

template<class Swarm>
void update_glb_best(int sw_best_idx, const Swarm &sw)
{
  update_glb_best_kernel<Swarm><<<1,1>>>(sw_best_idx,sw);
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
(float2, const EDTMap&, const E&, const M&, const C& , const S&);

#define INST_iterate_particles_de(M,C,E,S) template void PSO::iterate_particles_de< M,C,E,S > \
(const EDTMap&, const E&, const M&, const C& , const S&);

#define INST_setup_random_states(S) template void PSO::setup_random_states< S > \
(const S&);

#define INST_copy_best_values(S) template void PSO::copy_best_values< S > \
(float *best_values, const S&);\
template void PSO::update_glb_best< S > \
(int, const S&);

#define INST_group(M,C,E,S) INST_initialize_particles(M,C,E,S) \
  INST_iterate_particles(M,C,E,S) \
  INST_iterate_particles_de(M,C,E,S) \
  INST_setup_random_states(S) \
  INST_copy_best_values(S)

INST_group(UGV::UGVModel, UGV::UGVDPControl, UGV::NF1Evaluator, UGV::UGVSwarm<1>);
INST_group(UGV::UGVModel, UGV::UGVDPControl, UGV::NF1Evaluator, UGV::UGVSwarm<2>);
INST_group(UGV::UGVModel, UGV::UGVDPControl, UGV::NF1Evaluator, UGV::UGVSwarm<3>);
INST_group(UGV::UGVModel, UGV::UGVDPControl, UGV::NF1Evaluator, UGV::UGVSwarm<4>);
INST_group(UGV::UGVModel, UGV::UGVDPControl, UGV::NF1Evaluator, UGV::UGVSwarm<8>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVJLTControl, UGV::NF1Evaluator, UGV::UGVSwarm<1>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVJLTControl, UGV::NF1Evaluator, UGV::UGVSwarm<1>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVJLTControl, UGV::NF1Evaluator, UGV::UGVSwarm<2>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVJLTControl, UGV::NF1Evaluator, UGV::UGVSwarm<2>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::CorridorEvaluator, UGV::UGVSwarm<2>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::CorridorEvaluator, UGV::UGVSwarm<2>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::RefTrajEvaluator, UGV::UGVSwarm<2>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::RefTrajEvaluator, UGV::UGVSwarm<2>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::CorridorEvaluator, UGV::UGVSwarm<3>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::CorridorEvaluator, UGV::UGVSwarm<3>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::RefTrajEvaluator, UGV::UGVSwarm<3>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::RefTrajEvaluator, UGV::UGVSwarm<3>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::RefTrajEvaluator, UGV::UGVSwarm<4>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::RefTrajEvaluator, UGV::UGVSwarm<4>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::RefTrajEvaluator, UGV::UGVSwarm<1>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::RefTrajEvaluator, UGV::UGVSwarm<1>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::PPEvaluator, UGV::UGVSwarm<8>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::PPEvaluator, UGV::UGVSwarm<8>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::HybridEvaluator, UGV::UGVSwarm<1>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::HybridEvaluator, UGV::UGVSwarm<1>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::HybridEvaluator, UGV::UGVSwarm<2>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::HybridEvaluator, UGV::UGVSwarm<2>);

INST_initialize_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::HybridEvaluator, UGV::UGVSwarm<3>);
INST_iterate_particles(UGV::UGVModel, UGV::UGVDPControl, UGV::HybridEvaluator, UGV::UGVSwarm<3>);
