#ifndef PSO_KERNELS_CUH
#define PSO_KERNELS_CUH
#include <cpc_motion_planning/uav/uav_model.h>
#include <cpc_motion_planning/uav/uav_dp_control.h>
#include <cpc_motion_planning/uav/uav_swarm.h>
#include <cpc_motion_planning/uav/uav_single_target_evluator.h>
#include <cpc_motion_planning/uav/uav_jlt_control.h>
#include <cuda_geometry/cuda_edtmap.cuh>
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
void iterate_particles(float weight,
                       const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const Swarm &sw);

//---------
template<class Swarm>
void copy_best_values(float *best_values, const Swarm &sw);

////---------
//float evaluate_trajectory_wrapper(const State &s0, const Trace &tr, VoidPtrCarrier ptr_car,const UniformBinCarrier &ubc,
//               const EDTMap &map, const Trace &last_tr);
}
#endif // PSO_KERNELS_CUH
