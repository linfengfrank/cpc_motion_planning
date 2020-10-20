#ifndef UGV_DP_CONTROL_H
#define UGV_DP_CONTROL_H
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <vector>
namespace UGV
{
class UGVDPControl
{
public:
  UGVDPControl()
  {
         weight_prefix="/home/ugv/yzchen_ws/omen_deploy/UGV_new/";
        
//       weight_prefix="/home/sp/cpc_ws/ugv_new/";
  }

  ~UGVDPControl()
  {

  }


  void load_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {
    S_A = static_cast<CUDA_MAT::Matrix<4,UGVModel::Input>*>(factory.load_cuda_matrix<4,UGVModel::Input>(weight_prefix+"SA.dat",load_to_host));
    factory.load_uniform_bin(weight_prefix+"pos_bin.dat",ubc.bins[0]);
    factory.load_uniform_bin(weight_prefix+"vel_bin.dat",ubc.bins[1]);
    factory.load_uniform_bin(weight_prefix+"theta_bin.dat",ubc.bins[2]);
    factory.load_uniform_bin(weight_prefix+"w_bin.dat",ubc.bins[3]);
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

    UGVModel::Input u = CUDA_MAT::get_control_uniform_bin_4(s_relative, *S_A, ubc);
    return make_float3(u.acc,u.alpha,0.0f);
  }

  template<class Model, class Evaluator, class Swarm>
  __host__ __device__
  float simulate_evaluate(const EDTMap &map, const Evaluator &eva, Model &m, const Swarm &sw, const typename Swarm::Trace &ttr, bool &collision)
  {
    typename Model::State s_cmd = m.get_ini_state();
    typename Model::State s_slip = s_cmd;
    float cost = 0;
    float dt = PSO::PSO_SIM_DT;
    collision = false;
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      float3 u = dp_control(s_cmd, ttr[i]);
      m.model_forward(s_cmd,u,dt);
      m.model_forward_with_slip(s_slip,u,dt);

      //cost += 0.1f*sqrtf(u.x*u.x + 0.5f*u.y*u.y + u.z*u.z);
      cost += eva.process_cost(s_slip,map,t,collision);

    }
    cost += eva.final_cost(s_slip,map);

    return cost;
  }

  template<class Model, class Swarm>
  __host__ __device__
  std::vector<typename Model::State> generate_trajectory(Model &m, const Swarm &sw, const typename Swarm::Trace &ttr)
  {
    std::vector<typename Model::State> traj;
    typename Model::State s = m.get_ini_state();
    float dt = PSO::PSO_CTRL_DT;;
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

  CUDA_MAT::Matrix<4,UGVModel::Input> *S_A;

  UniformBinCarrier ubc;
  std::string weight_prefix;
};
}
#endif // UGV_DP_CONTROL_H
