#ifndef UAV_REPULSIVE_FIELD_H
#define UAV_REPULSIVE_FIELD_H
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/uav/uav_model.h>
#include <vector>
namespace UAV
{
struct RepulseRegister
{
  float3 force;
  bool valid;
};

void calc_repulsive_forces(const EDTMap &map, RepulseRegister* rgs, const UAV::UAVModel::State &s, unsigned int N);

class UAVRepulsiveField
{
public:
  UAVRepulsiveField(unsigned int N = 30):m_N(N)
  {
    m_hst_rgs = new RepulseRegister[N];
    CUDA_ALLOC_DEV_MEM(&m_dev_rgs,N*sizeof(RepulseRegister));
    CUDA_MEMCPY_H2D(m_dev_rgs,m_hst_rgs,N*sizeof(RepulseRegister));
  }

  ~UAVRepulsiveField()
  {
    delete [] m_hst_rgs;
    CUDA_FREE_DEV_MEM(m_dev_rgs);
  }


  float3 update_force(const EDTMap &map, UAV::UAVModel::State s)
  {
    calc_repulsive_forces(map, m_dev_rgs, s, m_N);
    CUDA_MEMCPY_D2H(m_hst_rgs,m_dev_rgs,m_N*sizeof(RepulseRegister));
    float3 F = make_float3(0,0,0);
    int valid_N = 0;
    for (unsigned int i=0; i<m_N; i++)
    {
      if (m_hst_rgs[i].valid)
      {
        F += m_hst_rgs[i].force;
        valid_N++;
      }
    }

    if (valid_N > 0)
    {
      F = F/static_cast<float>(valid_N);
    }

    // If far away from any obstacle, then try reduce speed to hover
    if (sqrtf(dot(F,F)) < 1.0f)
      F = -s.v;

    limit_norm(F,10.0f);

    return F;
  }

  void generate_repulse_traj(std::vector<UAV::UAVModel::State> &traj, const EDTMap &map, UAV::UAVModel::State s)
  {
    float3 F = update_force(map, s);
    traj.clear();
    float dt = PSO::PSO_CTRL_DT;

    limit_norm(s.v,3.0f);
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      state_forward(s,F,dt);
      traj.push_back(s);

      F = update_force(map, s);
    }
  }

  void state_forward(UAV::UAVModel::State &s, const float3 &F, const float &dt)
  {
    s.p = s.p + s.v*dt + 0.5f*F*dt*dt;
    s.v = s.v + F*dt;
    limit_norm(s.v,3.0f);
    s.a = make_float3(0,0,0);
  }

  void limit_norm(float3 &val, const float &max_norm)
  {
    float norm = sqrtf(dot(val,val));
    if (norm > max_norm)
      val = val/norm*max_norm;
  }

  RepulseRegister* m_hst_rgs;
  RepulseRegister* m_dev_rgs;
  unsigned int m_N;
};


}
#endif // UAV_REPULSIVE_FIELD_H
