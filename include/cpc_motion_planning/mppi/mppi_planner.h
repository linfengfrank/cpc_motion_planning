#ifndef MPPI_PLANNER_H
#define MPPI_PLANNER_H

#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/mppi/mppi_kernels.cuh>

namespace MPPI {
    template<class Model, class Controller, class Evaluator, class  PICore>
    class Planner {
    public:
        Planner(int num_of_sampling = 2560) {
            m_core.intepath_size = num_of_sampling;
        }

        ~Planner() {
        }

//        std::vector<typename Model::State> generate_trajectory() {
//            return
//        }

        void plan(const EDTMap &map) {
            cublasStatus_t cbls_stt;

            CUDA_MEMCPY_D2H(&result, m_core.initial_ctrl, sizeof(typename PICore::Trace));
            cudaDeviceSynchronize();
        }

        void initialize() {
            create_sample_seed();
        }

        void release() {
            delete_sample_seed();
        }

        void create_sample_seed() {
            typename PICore::IntegralPath* tIP = new typename PICore::IntegralPath[m_core.intepath_size];
            CUDA_ALLOC_DEV_MEM(&m_core.intepaths, m_core.intepath_size*sizeof(typename PICore::IntegralPath));
            CUDA_MEMCPY_H2D(m_core.intepaths, tIP, m_core.intepath_size*sizeof(typename PICore::IntegralPath));
            delete [] tIP;

            typename PICore::Trace* tcs = new typename PICore::Trace;
            CUDA_ALLOC_DEV_MEM(&m_core.nominal_ctrl_seq, sizeof(typename PICore::Trace));
            CUDA_MEMCPY_H2D(m_core.nominal_ctrl_seq, tcs, sizeof(typename PICore::Trace));

            CUDA_ALLOC_DEV_MEM(&m_core.initial_ctrl_seq, sizeof(typename PICore::Trace));
            CUDA_MEMCPY_H2D(m_core.initial_ctrl_seq, tcs, sizeof(typename PICore::Trace));
            delete [] tcs;

//            std::vector<float> costs(m_core.intepath_size, 0.0f);
            CUDA_ALLOC_DEV_MEM(&m_core.costs, m_core.intepath_size* sizeof(float));
//            CUDA_MEMCPY_H2D(m_core.costs, costs.data(), m_core.intepath_size* sizeof(float));

            CUDA_ALLOC_DEV_MEM(&m_core.exp_costs, m_core.intepath_size* sizeof(float));

            setup_random_sampling<PICore>(m_core);

            cublasCreate(&m_cbls_hdl);
        }

        void delete_particles() {
            CUDA_FREE_DEV_MEM(m_core.intepaths);
            CUDA_FREE_DEV_MEM(m_core.nominal_ctrl_seq);
            CUDA_FREE_DEV_MEM(m_core.initial_ctrl_seq);
            CUDA_FREE_DEV_MEM(m_core.costs);
            CUDA_FREE_DEV_MEM(m_core.exp_costs);
            cublasDestroy(m_cbls_hdl);
        }

    public:
        cublasHandle_t m_cbls_hdl;
        typename PICore::Trace result;
        Evaluator m_eva;
        Model m_model;
        Controller m_ctrl_dev;
        Controller m_ctrl_host;
        PICore m_core;
    };

}
#endif