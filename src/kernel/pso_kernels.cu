#include <cpc_motion_planning/pso/pso_kernels.cuh>
#include <cuda_geometry/helper_math.h>
#include <cpc_motion_planning/uav/uav_single_target_evluator.h>
namespace PSO
{
//---
template<class Model, class Controler, class Evaluator, class Swarm>
__host__ __device__
float evaluate_trajectory(const EDTMap &map, const Evaluator &eva, Model &m, const Controler &ctrl, const Swarm &sw, const typename Swarm::Trace &ttr)
{
  typename Model::State s = m.get_ini_state();
  float cost = 0;
  float dt = PSO_SIM_DT;
  //float3 goal_p = goal.s.p;
  for (float t=0.0f; t<PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/sw.step_dt));
    if (i > sw.steps - 1)
      i = sw.steps - 1;

    float3 u = ctrl.dp_control(s, ttr[i]);
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
template<class Swarm>
__global__
void setup_random_states_kernel(typename Swarm::Particle* tptcls)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  curand_init(9876, idx, 0, &(tptcls[idx].rs));
}

//---
template<class Model, class Controler, class Evaluator, class Swarm>
__global__
void initialize_particles_kernel(bool first_run,
                                 EDTMap map, Evaluator eva, Model m, Controler ctrl, Swarm sw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (first_run || idx != sw.ptcl_size-1)
  {
    sw.initialize_a_particle(m.get_ini_state(),sw.ptcls[idx]);
  }
  sw.ptcls[idx].best_cost = evaluate_trajectory<Model,Controler,Evaluator,Swarm>(map,eva,m, ctrl, sw, sw.ptcls[idx].best_loc);
}

//---
template<class Model, class Controler, class Evaluator, class Swarm>
__global__
void iterate_particles_kernel(float weight,
                              EDTMap map, Evaluator eva, Model m, Controler ctrl, Swarm sw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (idx == sw.ptcl_size-1)
    return;

  float r1 = sw.rand_float_gen(&(sw.ptcls[idx].rs),0,1);
  float r2 = sw.rand_float_gen(&(sw.ptcls[idx].rs),0,1);

  sw.ptcls[idx].ptcl_vel =
      sw.ptcls[idx].ptcl_vel*weight -
      (sw.ptcls[idx].curr_loc - sw.ptcls[idx].best_loc)*r1 -
      (sw.ptcls[idx].curr_loc - sw.ptcls[sw.ptcl_size-1].curr_loc)*r2;

  sw.bound_ptcl_velocity(sw.ptcls[idx]);

  sw.ptcls[idx].curr_loc = sw.ptcls[idx].curr_loc + sw.ptcls[idx].ptcl_vel;

  sw.bound_ptcl_location(m.get_ini_state(), sw.ptcls[idx]);

  float cost = evaluate_trajectory<Model,Controler,Evaluator,Swarm>(map,eva,m,ctrl,sw,sw.ptcls[idx].curr_loc);

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
template<class Model, class Controler, class Evaluator, class Swarm>
void initialize_particles(bool first_run,
                          const EDTMap &map,const Evaluator &eva, const Model &m, const Controler &ctrl, const Swarm &sw)
{
  initialize_particles_kernel<Model,Controler,Evaluator,Swarm><<<1,sw.ptcl_size>>>(first_run,map,eva,m,ctrl,sw);
}

//---------
template<class Model, class Controler, class Evaluator, class Swarm>
void iterate_particles(float weight,
                       const EDTMap &map, const Evaluator &eva, const Model &m, const Controler &ctrl, const Swarm &sw)
{
  iterate_particles_kernel<Model,Controler,Evaluator,Swarm><<<1,sw.ptcl_size>>>(weight,map,eva,m,ctrl,sw);
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

template void PSO::initialize_particles<UAV::UAVModel, UAV::UAVDPControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1> >(bool first_run,
                                                                       const EDTMap &map, const UAV::SingleTargetEvaluator &eva, const UAV::UAVModel &m, const UAV::UAVDPControl &ctrl , const  UAV::UAVSwarm<1> &sw);


template void PSO::iterate_particles<UAV::UAVModel,  UAV::UAVDPControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1> >(float weight,
                                                                    const EDTMap &map,const UAV::SingleTargetEvaluator &eva, const UAV::UAVModel &m,const UAV::UAVDPControl &ctrl , const  UAV::UAVSwarm<1> &sw);

template float PSO::evaluate_trajectory<UAV::UAVModel,  UAV::UAVDPControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1> >(
                          const EDTMap &map, const UAV::SingleTargetEvaluator &eva, UAV::UAVModel &m,const UAV::UAVDPControl &ctrl, const UAV::UAVSwarm<1> &sw, const UAV::UAVSwarm<1>::Trace &ttr);

template void PSO::setup_random_states< UAV::UAVSwarm<1> >(const UAV::UAVSwarm<1> &sw);

template void PSO::copy_best_values< UAV::UAVSwarm<1> >(float *best_values, const UAV::UAVSwarm<1> &sw);
