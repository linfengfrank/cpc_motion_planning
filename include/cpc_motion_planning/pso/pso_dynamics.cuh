#ifndef PSO_DYNAMICS_CUH
#define PSO_DYNAMICS_CUH

#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/pso/uav_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/helper_math.h>
#include <cuda_geometry/cuda_edtmap.cuh>
namespace PSO
{
struct Target
{
  UAVModel::State s;
  bool oa;
};

__host__ __device__ __forceinline__
float3 dp_control(const UAVModel::State &s, const float3 &site, VoidPtrCarrier &aux_data, const UniformBinCarrier &ubc)
{
  // Cast all the needed pointers
  CUDA_MAT::Mat3Act *S_A = static_cast<CUDA_MAT::Mat3Act*>(aux_data[0]);

  dp_action u[3];
  float s_relative[3];

  // X axis
  s_relative[0] = s.p.x - site.x; // relative position
  s_relative[1] = s.v.x; // relative velocity
  s_relative[2] = s.a.x; // relative velocity
  u[0] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A, ubc);

  //printf("%f \n",s_relative[0]);

  // Y axis
  s_relative[0] = s.p.y - site.y; // relative position
  s_relative[1] = s.v.y; // relative velocity
  s_relative[2] = s.a.y; // relative velocity
  u[1] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A, ubc);

  // Z axis
  s_relative[0] = s.p.z - site.z; // relative position
  s_relative[1] = s.v.z; // relative velocity
  s_relative[2] = s.a.z; // relative velocity
  u[2] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A, ubc);

  return make_float3(u[0].jerk, u[1].jerk, u[2].jerk);
}
}
#endif // PSO_DYNAMICS_CUH
