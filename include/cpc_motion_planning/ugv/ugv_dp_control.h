#ifndef UGV_DP_CONTROL_H
#define UGV_DP_CONTROL_H
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/ugv/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
namespace UGV
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

  void load_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {
    S_A = static_cast<CUDA_MAT::Matrix<4,UGVModel::Input>*>(factory.load_cuda_matrix<4,UGVModel::Input>("/home/sp/cpc_ws/slow/SA.dat",load_to_host));
    factory.load_uniform_bin("/home/sp/cpc_ws/slow/pos_bin.dat",bins[0]);
    factory.load_uniform_bin("/home/sp/cpc_ws/slow/vel_bin.dat",bins[1]);
    factory.load_uniform_bin("/home/sp/cpc_ws/slow/theta_bin.dat",bins[2]);
    factory.load_uniform_bin("/home/sp/cpc_ws/slow/w_bin.dat",bins[3]);
  }

  void release_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_from_host)
  {
    factory.free_cuda_matrix<4,UGVModel::Input>(S_A, load_from_host);
  }

  __host__ __device__
  float3 dp_control(const UGVModel::State &s, const float3 &site) const
  {
    // Construct the relative state
    float s_relative[4];
    s_relative[0] = s.s - site.x; // relative station
    s_relative[1] = s.v; // relative velocity
    s_relative[2] = s.theta - site.y; // relative velocity
    s_relative[3] = s.w; //relative angular speed

    UGVModel::Input u = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A, ubc);
  }
  CUDA_MAT::Matrix<4,UGVModel::Input> *S_A;

  UniformBinCarrier ubc;
};
}
#endif // UGV_DP_CONTROL_H
