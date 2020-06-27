#ifndef UAV_SIMPLE_CONTROL_H
#define UAV_SIMPLE_CONTROL_H
#include <cpc_motion_planning/mppi/mppi_utilities.cuh>
#include <cpc_motion_planning/uav/uav_model.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>

namespace UAV {
    class UAVSIMPLEControl {
    public:
        UAVSIMPLEControl() {

        }
        ~UAVSIMPLEControl() {

        }

        void load_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host) {

        }

        void set_limit(const float2 &vM, const float2 &aM, const float2 &jM) {

        }

        void release_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_from_host)
        {

        }

        template<class Model, class Evaluator, class PICore>
        __host__ __device__
        float simulate_evaluate(const EDTMap &map, const Evaluator &eva, Model &m, const PICore &core, const typename PICore::Trace &ttr, bool &collision) {
            typename Model::State s = m.get_ini_state();
            float cost = 0;
            float dt = MPPI::MPPI_SIM_DT;
            int curr_site_idx = -1;
            collision = false;
            for (float t = dt; t < MPPI::MPPI_TOTAL_T; t+= dt) {
                int i = static_cast<int>(floor(t/core.step_dt));
                if (i > core.steps - 1)
                    i = core.steps - 1;

//                printf("[%.2f] u: %.2f %.2f %.2f\n", t, ttr.site[curr_site_idx-1].x,
//                        ttr.site[curr_site_idx-1].y,
//                       ttr.site[curr_site_idx-1].z
//                );

//                printf("t: %.2f, u_id: %d\n", t, curr_site_idx);
                float propagate_dt = dt;
                if (i > curr_site_idx) {
                    curr_site_idx = i;
                    propagate_dt = t - curr_site_idx*core.step_dt;
                    float previous_propagate_dt = dt - propagate_dt;
                    if (curr_site_idx > 0) {
//                        printf("prev_dt: %.2f, u_id: %d\n", previous_propagate_dt, curr_site_idx-1);
                        m.model_forward(s, ttr.site[curr_site_idx-1], previous_propagate_dt);
                    }
                }
//                printf("propa_dt: %.2f, u_id: %d\n", propagate_dt, curr_site_idx);
                m.model_forward(s, ttr.site[curr_site_idx], propagate_dt);

//                printf("[%.2f] s: %.2f %.2f %.2f\n", t, s.p.x, s.p.y, s.p.z);
                cost += eva.process_cost(s, map, t, collision);
            }
            cost += eva.final_cost(s, map);
            return cost;
        }


    };
}
#endif
