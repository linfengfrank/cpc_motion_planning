#include <cpc_motion_planning/mppi/mppi_kernels.cuh>
#include <cuda_geometry/helper_math.h>

//#define VISUAL_DEBUG

namespace MPPI {
    // ------------- kernel function ---------- //
    template <class PICore>
    __global__
    void setup_random_sampling_kernel(typename PICore::IntegralPath* tintepts) {
        int idx = threadIdx.x+blockDim.x*blockIdx.x;
        curand_init(9876, idx, 0, &(tintepts[idx].rs));
    }

    template <class Model, class Controller, class Evaluator, class PICore>
    __global__
    void integral_paths_kernel(EDTMap map, Evaluator eva, Model m, Controller ctrl, PICore core) {
        int idx = threadIdx.x+blockDim.x*blockIdx.x;

        core.random_sample_du(core.intepaths[idx], m.var, m.limits);

        float cost = ctrl.template simulate_evaluate<Model, Evaluator, PICore>(map, eva, m, core,
                core.intepaths[idx].u, core.intepaths[idx].collision);

        float ctrl_cost = core.control_cost(core.intepaths[idx].u, m.var);
#ifdef VISUAL_DEBUG
        printf("[%d] cost: %.2f, ctrl_cost: %.2f\n",idx, cost, ctrl_cost);
#endif
        cost += ctrl_cost;
        core.costs[idx] = cost;
    }

    template <class PICore>
    __global__
    void calculate_exp_kernel(PICore core, int baseline_id) {
        int idx = threadIdx.x+blockDim.x*blockIdx.x;

        core.exp_costs[idx] = exp(-(core.costs[idx] - core.costs[baseline_id])/(core.lambda+0.001f)); // add eps for safety
#ifdef VISUAL_DEBUG
        printf("[%d] cost: %.2f, baseline_id: %d, min_cost:%.2f\n", idx, core.costs[idx], baseline_id, core.costs[baseline_id]);
        printf("[%d] exp_cost: %.2f\n", idx, core.exp_costs[idx]);
#endif
    }

    template <class PICore>
    __global__
    void calculate_weighted_update_kernal(PICore core, float eta) {
        int idx = threadIdx.x+blockDim.x*blockIdx.x;
        if (idx < core.steps) {
            for (int i = 0; i < core.intepath_size; i++) {
                float w = core.exp_costs[i] / eta;
                core.initial_ctrl_seq->site[idx] += w*core.intepaths[i].du[idx];
            }
        }
    }
    // ------------- launch function ---------- //
    template <class PICore>
    void setup_random_sampling(const PICore &core) {
        setup_random_sampling_kernel<PICore><<<1, core.intepath_size>>>(core.intepaths);
    }

    template<class Model, class Controller, class Evaluator, class PICore>
    void integral_paths(const EDTMap &map, const Evaluator &eva, const Model &m,
            const Controller &ctrl, const PICore &core) {
        integral_paths_kernel<Model, Controller, Evaluator, PICore><<<1, core.intepath_size>>>(map, eva, m, ctrl, core);
    }

    template <class PICore>
    void calculate_exp(const PICore &core, int baseline_id) {
        calculate_exp_kernel<PICore><<<1, core.intepath_size>>>(core, baseline_id);
    }

    template<class PICore>
    void calculate_weighted_update(const PICore &core, float eta) {
        calculate_weighted_update_kernal<PICore><<<1, core.steps>>>(core, eta);
    }
}

#define INST_setup_random_sampling(P) template void MPPI::setup_random_sampling< P > \
        (const P&);

#define INST_integral_paths(M,C,E,P) template void MPPI::integral_paths< M,C,E,P > \
        (const EDTMap&, const E&, const M&, const C&, const P&);

#define INST_calculate_exp(P) template void MPPI::calculate_exp< P > \
        (const P&, int);

#define INST_calculate_weighted_update(P) template void MPPI::calculate_weighted_update< P > \
        (const P&, float);

#define INST_group(M,C,E,P)  INST_setup_random_sampling(P) \
     INST_integral_paths(M,C,E,P) \
     INST_calculate_exp(P) \
     INST_calculate_weighted_update(P)

     INST_group(UAV::UAVModel, UAV::UAVSIMPLEControl, UAV::SingleTargetEvaluator, UAV::UAVMppi<35>);


