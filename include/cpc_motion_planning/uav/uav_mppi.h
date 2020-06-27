#ifndef UAV_MPPI_H
#define UAV_MPPI_H
#include <curand_kernel.h>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/uav/uav_model.h>
#include <cpc_motion_planning/mppi/mppi_utilities.cuh>
namespace UAV
{
    template <int STEP>
    class UAVMppi {
    public:
        struct Trace {
            float3 site[STEP];

            __device__ __host__
            Trace()
            {
                for (unsigned int i=0;i<STEP;i++)
                {
                    site[i] = make_float3(0,0,0);
                }
            }

            __device__ __host__
            float3& operator[](unsigned int i)
            {
                return site[i];
            }

            __device__ __host__
            const float3& operator[](unsigned int i) const
            {
                return site[i];
            }

            __device__ __host__
            Trace operator-(Trace const &rhs) const
            {
                Trace tmp;
                for (unsigned int i=0;i<STEP;i++)
                {
                    tmp[i] = site[i] - rhs[i];
                }
                return tmp;
            }
            //---
            __device__ __host__
            Trace operator+(Trace const &rhs) const
            {
                Trace tmp;
                for (unsigned int i=0;i<STEP;i++)
                {
                    tmp[i] = site[i] + rhs[i];
                }
                return tmp;
            }
            //---
            __device__ __host__
            Trace operator*(float const &rhs) const
            {
                Trace tmp;
                for (unsigned int i=0;i<STEP;i++)
                {
                    tmp[i] = site[i] * rhs;
                }
                return tmp;
            }
            //---
            __device__ __host__
            Trace operator/(float const &rhs) const
            {
                Trace tmp;
                for (unsigned int i=0;i<STEP;i++)
                {
                    tmp[i] = site[i] / rhs;
                }
                return tmp;
            }
            //---
            __device__ __host__
            Trace& operator=(Trace const &rhs)
            {
                for (unsigned int i=0;i<STEP;i++)
                {
                    site[i] = rhs[i];
                }
                return *this;
            }

            //---
            __device__ __host__
            float square()
            {
                float square = 0;
                for (unsigned int i=0;i<STEP;i++)
                {
                    square += dot(site[i],site[i]);
                }
                return square;
            }
        };

        struct IntegralPath{
            curandState rs;
            Trace du;
            Trace u;
            bool collision;
            float weight;
        };

        UAVMppi() {
            steps = STEP;
            step_dt = MPPI::MPPI_TOTAL_T/static_cast<float>(steps);
        }

        ~UAVMppi() {

        }

        __device__
        void random_sample_du(IntegralPath &intepath, float3 var) {
            for (int i = 0 ; i < STEP; i++) {
                float r = MPPI::rand_float_gen(&(intepath.rs), -1.0, 1.0);
                intepath.du.site[i].x = var.x * r;
                r = MPPI::rand_float_gen(&(intepath.rs), -1.0, 1.0);
                intepath.du.site[i].y = var.y * r;
                r = MPPI::rand_float_gen(&(intepath.rs), -1.0, 1.0);
                intepath.du.site[i].z = var.z * r;
                intepath.u.site[i] = initial_ctrl_seq->site[i] + intepath.du.site[i];
            }
        }

        __device__
        float control_cost(const Trace& cs, const float3& var) {
            float cost = 0.0;
            for (int i = 0; i < steps; i++) {
                float3 delta_u = (*initial_ctrl_seq)[i] - (*nominal_ctrl_seq)[i];
                cost += delta_u.x * cs[i].x / var.x;
                cost += delta_u.y * cs[i].y / var.y;
                cost += delta_u.z * cs[i].z / var.z;
            }
            cost *= lambda;
            return cost;
        }

        __host__
        void set_nominal_and_initial_cs(Trace n_cs, Trace i_cs) {
            CUDA_MEMCPY_H2D(nominal_ctrl_seq, &n_cs, sizeof(Trace));
            CUDA_MEMCPY_H2D(initial_ctrl_seq, &i_cs, sizeof(Trace));
        }

        // --- elements

        IntegralPath *intepaths; // N_sample size of Integral
        Trace *nominal_ctrl_seq; // 1 size of Trace
        Trace *initial_ctrl_seq; // 1 size of Trace

        float *costs; // N_sample size of float

        float *contrl_costs; // N_sample size of float

        float *exp_costs; // N_sample size of float

        int intepath_size; // number of samples
        int steps;
        float step_dt;

        float lambda;
    };
}

#endif
