#ifndef MPPI_PLANNER_H
#define MPPI_PLANNER_H

#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/mppi/mppi_kernels.cuh>

#define TRAJ_VISUAL
#ifdef TRAJ_VISUAL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#define VISUAL_NUM 10
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
#endif


namespace MPPI {
    template<class Model, class Controller, class Evaluator, class  PICore>
    class Planner {
    public:
        Planner(int num_of_sampling = 2560, float lambda = 1.0): m_core(PICore(lambda)) {
            m_core.intepath_size = num_of_sampling;
            count_propagate_t = 0.0f;
#ifdef TRAJ_VISUAL
            SampleOut = PointCloud::Ptr(new PointCloud);
            SampleOut->header.frame_id = "/world";
            OptimalTrajOut = PointCloud::Ptr(new PointCloud);
            OptimalTrajOut->header.frame_id = "/world";
#endif
        }

        ~Planner() {
        }

        std::vector<typename Model::State> generate_trajectory(float start_t = 0) {
            return m_ctrl_host.template generate_trajectory<Model, PICore>(m_model, m_core, result, start_t);
        }

        bool shift_update(float dt) {
            bool updated = false;
            count_propagate_t += dt;
            while (count_propagate_t > m_core.step_dt) {
                count_propagate_t -= m_core.step_dt;
                float3 rand_u = make_float3(0,0,0);
                result.shift_update(rand_u);
                updated = true;
            }
            return updated;
        }

        void plan(const EDTMap &map) {
            cublasStatus_t cbls_stt;

            setup_random_sampling<PICore>(m_core);
//            cudaDeviceSynchronize();

            integral_paths<Model, Controller, Evaluator, PICore>(map, m_eva, m_model, m_ctrl_dev, m_core);
//            cudaDeviceSynchronize();

            int min_cost_idx = -1;
            cbls_stt = cublasIsamin(m_cbls_hdl, m_core.intepath_size, m_core.costs, 1, &min_cost_idx);
            int max_cost_idx = -1;
            cbls_stt = cublasIsamax(m_cbls_hdl, m_core.intepath_size, m_core.costs, 1, &max_cost_idx);

            typename PICore::Trace optimal_ctrl_seq;
            if (min_cost_idx != -1 && max_cost_idx != -1) {

                float min_cost, max_cost;
                CUDA_MEMCPY_D2H(&min_cost , m_core.costs+min_cost_idx-1, sizeof(float));
                CUDA_MEMCPY_D2H(&max_cost , m_core.costs+max_cost_idx-1, sizeof(float));
                printf("min_cost: %.2f,  max_cost: %.2f\n", min_cost, max_cost);
                float scale = m_core.delta_cost_limits.z / fminf(fmaxf(max_cost - min_cost, m_core.delta_cost_limits.x),
                        m_core.delta_cost_limits.y);
                scale = 1.0;
                printf("scale: %.2f\n", scale);
                calculate_exp<PICore>(m_core, min_cost_idx-1, scale);
//                cudaDeviceSynchronize();
                float eta = 0;
                cbls_stt = cublasSasum(m_cbls_hdl, m_core.intepath_size, m_core.exp_costs, 1, &eta);
                while (fabsf(eta) < 0.01) {
                    eta += (eta>0)? 0.01: -0.01;
                }
                printf("eta: %.6f\n", eta);
//                    cudaDeviceSynchronize();
                calculate_weighted_update<PICore>(m_core, eta);
                CUDA_MEMCPY_D2H(&optimal_ctrl_seq, m_core.initial_ctrl_seq, sizeof(typename PICore::Trace));
                bool collision = false;
                float optimal_cost = m_ctrl_host.template simulate_evaluate<Model,Evaluator,PICore>(map, m_eva, m_model, m_core, optimal_ctrl_seq, collision);
                printf("optimal_cost: %.2f\n", optimal_cost);
//                if (optimal_cost > min_cost || collision) {
//                    printf("reset to min cost csq\n");
//                    CUDA_MEMCPY_D2H(&optimal_ctrl_seq, &m_core.intepaths[min_cost_idx-1].u, sizeof(typename PICore::Trace));
//                }
            } else {
                return;
            }
            result = optimal_ctrl_seq;
//            result.print();
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
            CUDA_ALLOC_DEV_MEM(&m_core.ctrl_costs, m_core.intepath_size* sizeof(float));
//            CUDA_MEMCPY_H2D(m_core.costs, costs.data(), m_core.intepath_size* sizeof(float));

            CUDA_ALLOC_DEV_MEM(&m_core.exp_costs, m_core.intepath_size* sizeof(float));


            cublasCreate(&m_cbls_hdl);
        }

        void delete_sample_seed() {
            CUDA_FREE_DEV_MEM(m_core.intepaths);
            CUDA_FREE_DEV_MEM(m_core.nominal_ctrl_seq);
            CUDA_FREE_DEV_MEM(m_core.initial_ctrl_seq);
            CUDA_FREE_DEV_MEM(m_core.costs);
            CUDA_FREE_DEV_MEM(m_core.exp_costs);
            cublasDestroy(m_cbls_hdl);
        }

#ifdef TRAJ_VISUAL
        void visualize() {
            SampleOut->clear();
            OptimalTrajOut->clear();
            CUDA_ALLOC_DEV_MEM(&m_core.sample_states, (VISUAL_NUM)*m_core.steps*sizeof(float3));
            visualize_samples<PICore, Controller, Model>(m_core, m_ctrl_dev, m_model, VISUAL_NUM+1);
            std::vector<float3> sample_database;
            sample_database.resize((VISUAL_NUM)*m_core.steps);
            CUDA_MEMCPY_D2H(sample_database.data(), m_core.sample_states, (VISUAL_NUM)*m_core.steps*sizeof(float3));
            for (int i = 0; i < VISUAL_NUM; i++) {
                for (int j = 0; j < m_core.steps; j++) {
                    pcl::PointXYZRGB s_p;
                    s_p.x = sample_database[i*m_core.steps + j].x;
                    s_p.y = sample_database[i*m_core.steps + j].y;
                    s_p.z = sample_database[i*m_core.steps + j].z;
                    s_p.r = 255;
                    s_p.g = 255;
                    s_p.b = 255;
                    SampleOut->points.push_back(s_p);
                }
            }
            CUDA_FREE_DEV_MEM(m_core.sample_states);

            std::vector<typename Model::State> optimal_traj = m_ctrl_host.template generate_trajectory<Model, PICore>(m_model,
                    m_core, result);
            for (int i = 0; i < optimal_traj.size(); i++) {
                pcl::PointXYZRGB s_p;
                s_p.x = optimal_traj[i].p.x;
                s_p.y = optimal_traj[i].p.y;
                s_p.z = optimal_traj[i].p.z;
                s_p.r = 255;
                s_p.g = 0;
                s_p.b = 0;
                OptimalTrajOut->points.push_back(s_p);
            }
        }
#endif

    public:
        cublasHandle_t m_cbls_hdl;
        typename PICore::Trace result;
        float count_propagate_t;
        Evaluator m_eva;
        Model m_model;
        Controller m_ctrl_dev;
        Controller m_ctrl_host;
        PICore m_core;

#ifdef TRAJ_VISUAL
        PointCloud::Ptr SampleOut;
        PointCloud::Ptr OptimalTrajOut;
#endif

    };

}
#endif