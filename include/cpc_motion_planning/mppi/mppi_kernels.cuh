#ifndef MPPI_KERNELS_CUH
#define MPPI_KERNELS_CUH
#include <cpc_motion_planning/uav/uav_model.h>
#include <cpc_motion_planning/uav/uav_mppi.h>
#include <cpc_motion_planning/uav/uav_simple_control.h>
#include <cpc_motion_planning/uav/uav_single_target_evluator.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1map.cuh>
#include "cublas_v2.h"
namespace MPPI
{
    template<class PICore>
    void setup_random_sampling(const PICore &core);

    template<class Model, class Controller, class Evaluator, class PICore>
    void integral_paths(const EDTMap &map, const Evaluator &eva, const Model &m, const Controller &ctrl, const PICore &core);
    // --- output: cost values of paths

    // --- do cublasIsamin to find the minimum cost

    template<class PICore>
    void calculate_exp(const PICore &core, int baseline_id, float scale);
    // --- output: exp terms for paths

    // --- do cublasSasum to get eta

    template<class PICore>
    void calculate_weighted_update(const PICore &core, float eta);
    // --- output: weights for paths

    template<class PICore, class Controller, class Model>
    void visualize_samples(const PICore &core, const Controller &ctrl, const Model &model, int visual_num);

}
#endif