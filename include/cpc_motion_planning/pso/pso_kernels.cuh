#ifndef PSO_KERNELS_CUH
#define PSO_KERNELS_CUH
#include <cpc_motion_planning/uav/model/uav_model.h>
#include <cpc_motion_planning/uav/controller/uav_dp_control.h>
#include <cpc_motion_planning/uav/controller/uav_dp_vel_control.h>
#include <cpc_motion_planning/uav/swarm/uav_swarm.h>
#include <cpc_motion_planning/uav/evaluator/uav_single_target_evaluator.h>
#include <cpc_motion_planning/uav/evaluator/uav_nf1_evaluator.h>
#include <cpc_motion_planning/uav/evaluator/uav_corridor_nf2_evaluator.h>
#include <cpc_motion_planning/uav/controller/uav_jlt_control.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1map.cuh>
#include "cublas_v2.h"
namespace PSO
{
template<class Swarm>
void setup_random_states(const Swarm &sw);

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
void initialize_particles(bool first_run,
                          const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw);

//---------
template<class Model, class Controller, class Evaluator, class Swarm>
void iterate_particles(float2 weight,
                       const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw);

//---------
template<class Swarm>
void copy_best_values(float *best_values, const Swarm &sw);

//---------
template<class Swarm>
void update_glb_best(int sw_best_idx, const Swarm &sw);

////---------
//float evaluate_trajectory_wrapper(const State &s0, const Trace &tr, VoidPtrCarrier ptr_car,const UniformBinCarrier &ubc,
//               const EDTMap &map, const Trace &last_tr);
}
#endif // PSO_KERNELS_CUH
