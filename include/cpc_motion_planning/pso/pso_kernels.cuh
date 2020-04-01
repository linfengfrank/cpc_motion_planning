#ifndef PSO_KERNELS_CUH
#define PSO_KERNELS_CUH
#include <cpc_motion_planning/pso/pso_dynamics.cuh>
namespace PSO
{
void setup_random_states(void *ptcls, int size);
template<int N>
void test_plan(void *ptcls, int ptcls_size, bool first_run, VoidPtrCarrier<N> ptr_car);
//void initializeWrapper(const state &s, const float3 &goal, const Ptcl &p, const DevLocalMap &map, BSCP *bscp, NNDP_UTIL::FLY_STATUS fs, bool first_run);
//void iterateWrapper(const state &s, const float3 &goal, const Ptcl &p, float w, const DevLocalMap &map, BSCP *bscp, NNDP_UTIL::FLY_STATUS fs,bool reset);
}
#endif // PSO_KERNELS_CUH
