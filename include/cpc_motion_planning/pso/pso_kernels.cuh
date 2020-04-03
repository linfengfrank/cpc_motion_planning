#ifndef PSO_KERNELS_CUH
#define PSO_KERNELS_CUH
#include <cpc_motion_planning/pso/pso_dynamics.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include "cublas_v2.h"
namespace PSO
{
void setup_random_states(const Swarm &sw);

//---------
template<int N>
void initialize_particles(const Swarm &sw, bool first_run,
                          const State &s, const State &goal,VoidPtrCarrier<N> ptr_car, const  UniformBinCarrier &ubc,
                          const EDTMap &map, const Trace &last_tr);

//---------
template<int N>
void iterate_particles(const Swarm &sw, float weight,
                       const State &s, const State &goal,VoidPtrCarrier<N> ptr_car, const  UniformBinCarrier &ubc,
                       const EDTMap &map, const Trace &last_tr);

//---------
void copy_best_values(const Swarm &sw, float *best_values);
}
#endif // PSO_KERNELS_CUH
