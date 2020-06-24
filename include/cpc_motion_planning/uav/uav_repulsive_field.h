#ifndef UAV_REPULSIVE_FIELD_H
#define UAV_REPULSIVE_FIELD_H
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/uav/uav_model.h>
namespace UAV
{
void calc_repulsive_forces(const EDTMap &map, float3* rgs, const UAV::UAVModel::State &s, unsigned int N);

class UAVRepulsiveField
{
public:
  UAVRepulsiveField(unsigned int N = 30):m_N(N)
  {
    m_hst_rgs = new float3[N];
    CUDA_ALLOC_DEV_MEM(&m_dev_rgs,N*sizeof(float3));
    CUDA_MEMCPY_H2D(m_dev_rgs,m_hst_rgs,N*sizeof(float3));
  }

  ~UAVRepulsiveField()
  {
    delete [] m_hst_rgs;
    CUDA_FREE_DEV_MEM(m_dev_rgs);
  }


  float3 update_force(const EDTMap &map, const UAV::UAVModel::State &s)
  {
    calc_repulsive_forces(map, m_dev_rgs, s, m_N);
    CUDA_MEMCPY_D2H(m_hst_rgs,m_dev_rgs,m_N*sizeof(float3));
    float3 F = make_float3(0,0,0);
    for (unsigned int i=0; i<m_N; i++)
    {
      F += m_hst_rgs[i];
    }
    return F;
  }

  float3* m_hst_rgs;
  float3* m_dev_rgs;
  unsigned int m_N;
};


}
#endif // UAV_REPULSIVE_FIELD_H
