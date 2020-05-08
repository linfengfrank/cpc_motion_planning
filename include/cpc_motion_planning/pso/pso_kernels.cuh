#ifndef PSO_KERNELS_CUH
#define PSO_KERNELS_CUH
#include <cpc_motion_planning/uav/uav_model.h>
#include <cpc_motion_planning/uav/uav_dp_control.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include "cublas_v2.h"
namespace PSO
{
void setup_random_states(const Swarm &sw);

//---------
template<class Model, class Controler, class Evaluator>
void initialize_particles(const Swarm &sw, bool first_run,
                          const EDTMap &map, const Trace &last_tr, const Evaluator &eva, const Model &m, const Controler &ctrl);

//---------
template<class Model, class Controler, class Evaluator>
void iterate_particles(const Swarm &sw, float weight,
                       const EDTMap &map, const Trace &last_tr, const Evaluator &eva, const Model &m, const Controler &ctrl);

//---------
void copy_best_values(const Swarm &sw, float *best_values);

////---------
//float evaluate_trajectory_wrapper(const State &s0, const Trace &tr, VoidPtrCarrier ptr_car,const UniformBinCarrier &ubc,
//               const EDTMap &map, const Trace &last_tr);
}
#endif // PSO_KERNELS_CUH
