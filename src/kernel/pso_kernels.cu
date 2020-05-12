#include <cpc_motion_planning/pso/pso_kernels.cuh>
#include <cuda_geometry/helper_math.h>
#include <cpc_motion_planning/uav/uav_single_target_evluator.h>
namespace PSO
{
//---
template<class Model, class Controler, class Evaluator, class TmpSwarm>
__host__ __device__
float evaluate_trajectory(const EDTMap &map, const Evaluator &eva, Model &m, const Controler &ctrl, const typename TmpSwarm::Trace &ttr)
{
  typename Model::State s = m.get_ini_state();
  float cost = 0;
  float dt = PSO_SIM_DT;
  //float3 goal_p = goal.s.p;
  for (float t=0.0f; t<PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/PSO_STEP_DT));
    if (i > PSO_STEPS - 1)
      i = PSO_STEPS - 1;

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
template<class TmpSwarm>
__global__
void setup_random_states_kernel(typename TmpSwarm::Particle* tptcls)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  curand_init(9876, idx, 0, &(tptcls[idx].rs));
}

//---
template<class Model, class Controler, class Evaluator, class TmpSwarm>
__global__
void initialize_particles_kernel(bool first_run,
                                 EDTMap map, Evaluator eva, Model m, Controler ctrl, TmpSwarm tsw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (first_run || idx != tsw.ptcl_size-1)
  {
    tsw.initialize_a_particle(m.get_ini_state(),tsw.ptcls[idx]);
  }
  tsw.ptcls[idx].best_cost = evaluate_trajectory<Model,Controler,Evaluator,TmpSwarm>(map,eva,m, ctrl, tsw.ptcls[idx].best_loc);
}

//---
template<class Model, class Controler, class Evaluator, class TmpSwarm>
__global__
void iterate_particles_kernel(float weight,
                              EDTMap map, Evaluator eva, Model m, Controler ctrl, TmpSwarm tsw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;

  if (idx == tsw.ptcl_size-1)
    return;

  float r1 = tsw.rand_float_gen(&(tsw.ptcls[idx].rs),0,1);
  float r2 = tsw.rand_float_gen(&(tsw.ptcls[idx].rs),0,1);

  tsw.ptcls[idx].ptcl_vel =
      tsw.ptcls[idx].ptcl_vel*weight -
      (tsw.ptcls[idx].curr_loc - tsw.ptcls[idx].best_loc)*r1 -
      (tsw.ptcls[idx].curr_loc - tsw.ptcls[tsw.ptcl_size-1].curr_loc)*r2;

  tsw.bound_ptcl_velocity(tsw.ptcls[idx]);

  tsw.ptcls[idx].curr_loc = tsw.ptcls[idx].curr_loc + tsw.ptcls[idx].ptcl_vel;

  tsw.bound_ptcl_location(m.get_ini_state(), tsw.ptcls[idx]);

  float cost = evaluate_trajectory<Model,Controler,Evaluator,TmpSwarm>(map,eva,m,ctrl,tsw.ptcls[idx].curr_loc);

  if (cost < tsw.ptcls[idx].best_cost)
  {
    tsw.ptcls[idx].best_cost = cost;
    tsw.ptcls[idx].best_loc = tsw.ptcls[idx].curr_loc;
  }
}

//---------
template<class TmpSwarm>
__global__
void copy_best_value_kernel(float* best_values, TmpSwarm tsw)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  best_values[idx] = tsw.ptcls[idx].best_cost;
}

//---------
template<class TmpSwarm>
void setup_random_states(const TmpSwarm &tsw)
{
  setup_random_states_kernel<TmpSwarm><<<1,tsw.ptcl_size>>>(tsw.ptcls);
}

//---------
template<class Model, class Controler, class Evaluator, class TmpSwarm>
void initialize_particles(bool first_run,
                          const EDTMap &map,const Evaluator &eva, const Model &m, const Controler &ctrl, const TmpSwarm &tsw)
{
  initialize_particles_kernel<Model,Controler,Evaluator,TmpSwarm><<<1,tsw.ptcl_size>>>(first_run,map,eva,m,ctrl,tsw);
}

//---------
template<class Model, class Controler, class Evaluator, class TmpSwarm>
void iterate_particles(float weight,
                       const EDTMap &map, const Evaluator &eva, const Model &m, const Controler &ctrl, const TmpSwarm &tsw)
{
  iterate_particles_kernel<Model,Controler,Evaluator,TmpSwarm><<<1,tsw.ptcl_size>>>(weight,map,eva,m,ctrl,tsw);
}

//---------
template<class TmpSwarm>
void copy_best_values(float *best_values, const TmpSwarm &tsw)
{
  copy_best_value_kernel<TmpSwarm><<<1,tsw.ptcl_size>>>(best_values,tsw);
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
                                                                       const EDTMap &map, const UAV::SingleTargetEvaluator &eva, const UAV::UAVModel &m, const UAV::UAVDPControl &ctrl , const  UAV::UAVSwarm<1> &tsw);


template void PSO::iterate_particles<UAV::UAVModel,  UAV::UAVDPControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1> >(float weight,
                                                                    const EDTMap &map,const UAV::SingleTargetEvaluator &eva, const UAV::UAVModel &m,const UAV::UAVDPControl &ctrl , const  UAV::UAVSwarm<1> &tsw);

template float PSO::evaluate_trajectory<UAV::UAVModel,  UAV::UAVDPControl, UAV::SingleTargetEvaluator, UAV::UAVSwarm<1> >(
                          const EDTMap &map, const UAV::SingleTargetEvaluator &eva, UAV::UAVModel &m,const UAV::UAVDPControl &ctrl, const UAV::UAVSwarm<1>::Trace &ttr);

template void PSO::setup_random_states< UAV::UAVSwarm<1> >(const UAV::UAVSwarm<1> &tsw);

template void PSO::copy_best_values< UAV::UAVSwarm<1> >(float *best_values, const UAV::UAVSwarm<1> &tsw);
