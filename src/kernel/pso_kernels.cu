#include <cpc_motion_planning/pso/pso_kernels.cuh>

namespace PSO
{
__global__
void setup_random_states_kernel(void *ptcls)
{
    int idx = threadIdx.x+blockDim.x*blockIdx.x;
    curand_init(1234, idx, 0, &(static_cast<Particle*>(ptcls)[idx].rs));
    float myrandf = curand_uniform(&(static_cast<Particle*>(ptcls)[idx].rs));

    printf("%f\n",myrandf);
}
//---
void setup_random_states(void *ptcls, int size)
{
    setup_random_states_kernel<<<1,size>>>(ptcls);
}
}
