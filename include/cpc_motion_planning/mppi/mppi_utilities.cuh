#ifndef MPPI_CUH
#define MPPI_CUH
#include <curand_kernel.h>
#include <cuda_math/cuda_matrix.cuh>
namespace MPPI
{
    const float MPPI_TOTAL_T = 4.0f;
    const float MPPI_SIM_DT = 0.1f;
    const float MPPI_CTRL_DT = 0.05f;

    __host__ __device__ __forceinline__
    float rand_float_gen(curandState *rs, float min, float max)
    {
#ifdef  __CUDA_ARCH__
        float myrandf = curand_uniform(rs);
#else
        float myrandf = 0.0;
#endif
        myrandf *= (max - min);// + 0.999999f);
        myrandf += min;
        return myrandf;
    }

    __host__ __device__ __forceinline__
    void bound_between(float &val, float min, float max)
    {
        if (val < min)
            val = min;
        else if (val > max)
            val = max;
    }
}
#endif