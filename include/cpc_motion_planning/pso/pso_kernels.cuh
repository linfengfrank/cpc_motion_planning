#ifndef PSO_KERNELS_CUH
#define PSO_KERNELS_CUH
#include <cpc_motion_planning/pso/pso_dynamics.cuh>
#include "cublas_v2.h"
namespace PSO
{
void setup_random_states(Particle *ptcls, int size);

//---------
template<int N>
void initialize_particles(Particle *ptcls, int ptcls_size, bool first_run,
                          const State &s, const State &goal,VoidPtrCarrier<N> ptr_car);

//---------
template<int N>
void iterate_particles(Particle *ptcls, int ptcls_size, float weight,
                          const State &s, const State &goal,VoidPtrCarrier<N> ptr_car);

//---------
void copy_best_values(Particle *ptcls, int ptcls_size, float *best_values);
}
#endif // PSO_KERNELS_CUH
