#ifndef UGV_JLT_CONTROL_H
#define UGV_JLT_CONTROL_H
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/ugv/ugv_model.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/JLT.h>

namespace UGV
{
class UGVJLTControl
{
public:
  UGVJLTControl()
  {

  }

  ~UGVJLTControl()
  {

  }

  void load_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {

  }

  void release_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_from_host)
  {

  }

  __host__ __device__
  float3 dp_control(const UGVModel::State &s, const float3 &site) const
  {
    return make_float3(0,0,0);
  }

  JLT m_jlt_planner;
};
}
#endif // UGV_JLT_CONTROL_H
