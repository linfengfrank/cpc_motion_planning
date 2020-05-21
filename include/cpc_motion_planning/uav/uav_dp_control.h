#ifndef UAV_DP_CONTROL_H
#define UAV_DP_CONTROL_H
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/uav/uav_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <vector>

namespace UAV
{
class UAVDPControl
{
public:
  UAVDPControl():m_theta(0)
  {

  }

  ~UAVDPControl()
  {

  }

  void load_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {
    S_A_horizontal = static_cast<CUDA_MAT::Matrix<3,UAVModel::Input>*>(factory.load_cuda_matrix<3,UAVModel::Input>("/home/sp/cpc_ws/SA.dat",load_to_host));
    S_A_vertical = static_cast<CUDA_MAT::Matrix<3,UAVModel::Input>*>(factory.load_cuda_matrix<3,UAVModel::Input>("/home/sp/cpc_ws/SA.dat",load_to_host));
    factory.load_uniform_bin("/home/sp/cpc_ws/pos_bin.dat",ubc.bins[0]);
    factory.load_uniform_bin("/home/sp/cpc_ws/vel_bin.dat",ubc.bins[1]);
    factory.load_uniform_bin("/home/sp/cpc_ws/acc_bin.dat",ubc.bins[2]);
  }

  void release_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_from_host)
  {
    factory.free_cuda_matrix<3,UAVModel::Input>(S_A_horizontal, load_from_host);
    factory.free_cuda_matrix<3,UAVModel::Input>(S_A_vertical, load_from_host);
  }

  __host__ __device__
  float3 dp_control(const UAVModel::State &s, const float3 &site) const
  {
    UAVModel::Input u[3];
    float s_relative[3];

    // X axis
    s_relative[0] = s.p.x - site.x; // relative position
    s_relative[1] = s.v.x; // relative velocity
    s_relative[2] = s.a.x; // relative velocity
    u[0] = CUDA_MAT::get_control_uniform_bin_3(s_relative, *S_A_horizontal, ubc);

    //printf("%f \n",s_relative[0]);

    // Y axis
    s_relative[0] = s.p.y - site.y; // relative position
    s_relative[1] = s.v.y; // relative velocity
    s_relative[2] = s.a.y; // relative velocity
    u[1] = CUDA_MAT::get_control_uniform_bin_3(s_relative, *S_A_horizontal, ubc);

    // Z axis
    s_relative[0] = s.p.z - site.z; // relative position
    s_relative[1] = s.v.z; // relative velocity
    s_relative[2] = s.a.z; // relative velocity
    u[2] = CUDA_MAT::get_control_uniform_bin_3(s_relative, *S_A_horizontal, ubc);

    return make_float3(u[0].jerk, u[1].jerk, u[2].jerk);
  }

  template<class Model, class Evaluator, class Swarm>
  __host__ __device__
  float simulate_evaluate(const EDTMap &map, const Evaluator &eva, Model &m, const Swarm &sw, const typename Swarm::Trace &ttr)
  {
    typename Model::State s = m.get_ini_state();
    float cost = 0;
    float dt = PSO::PSO_SIM_DT;
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      float3 u = dp_control(s, ttr[i]);
      m.model_forward(s,u,dt);

      cost += 0.1*sqrt(u.x*u.x + u.y*u.y + u.z*u.z);
      cost += eva.process_cost(s,map);

    }
    cost += eva.final_cost(s,map);
    return cost;
  }

  template<class Model, class Swarm>
  __host__ __device__
  std::vector<typename Model::State> generate_trajectory(Model &m, const Swarm &sw, const typename Swarm::Trace &ttr)
  {
    std::vector<typename Model::State> traj;
    typename Model::State s = m.get_ini_state();
    float dt = PSO::PSO_CTRL_DT;
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      float3 u = dp_control(s, ttr[i]);
      m.model_forward(s,u,dt);
      traj.push_back(s);
    }
    return traj;
  }
  CUDA_MAT::Matrix<3,UAVModel::Input> *S_A_horizontal;
  CUDA_MAT::Matrix<3,UAVModel::Input> *S_A_vertical;
  UniformBinCarrier ubc;
  float m_theta;
};
}
#endif // UAV_DP_CONTROL_H
