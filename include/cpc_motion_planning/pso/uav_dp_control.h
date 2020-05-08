#ifndef UAV_DP_CONTROL_H
#define UAV_DP_CONTROL_H
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/pso/uav_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>

namespace PSO
{
class UAVDPControl
{
public:
  UAVDPControl()
  {

  }

  ~UAVDPControl()
  {

  }

  __host__ __device__
  float3 dp_control(const UAVModel::State &s, const float3 &site) const
  {
    dp_action u[3];
    float s_relative[3];

    // X axis
    s_relative[0] = s.p.x - site.x; // relative position
    s_relative[1] = s.v.x; // relative velocity
    s_relative[2] = s.a.x; // relative velocity
    u[0] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A_horizontal, ubc);

    //printf("%f \n",s_relative[0]);

    // Y axis
    s_relative[0] = s.p.y - site.y; // relative position
    s_relative[1] = s.v.y; // relative velocity
    s_relative[2] = s.a.y; // relative velocity
    u[1] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A_horizontal, ubc);

    // Z axis
    s_relative[0] = s.p.z - site.z; // relative position
    s_relative[1] = s.v.z; // relative velocity
    s_relative[2] = s.a.z; // relative velocity
    u[2] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A_horizontal, ubc);

    return make_float3(u[0].jerk, u[1].jerk, u[2].jerk);
  }
  CUDA_MAT::Mat3Act *S_A_horizontal;
  CUDA_MAT::Mat3Act *S_A_vertical;
  UniformBinCarrier ubc;
};
}
#endif // UAV_DP_CONTROL_H
