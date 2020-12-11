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

  }

  ~UGVDPControl()
  {

  }


  void load_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {
    S_A = static_cast<CUDA_MAT::Matrix<4,UGVModel::Input>*>(factory.load_cuda_matrix<4,UGVModel::Input>("/home/sp/cpc_ws/ugv_new/SA.dat",load_to_host));
    factory.load_uniform_bin("/home/sp/cpc_ws/ugv_new/pos_bin.dat",ubc.bins[0]);
    factory.load_uniform_bin("/home/sp/cpc_ws/ugv_new/vel_bin.dat",ubc.bins[1]);
    factory.load_uniform_bin("/home/sp/cpc_ws/ugv_new/theta_bin.dat",ubc.bins[2]);
    factory.load_uniform_bin("/home/sp/cpc_ws/ugv_new/w_bin.dat",ubc.bins[3]);
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
    s_relative[2] = s.theta - site.y; // relative angle
    s_relative[3] = s.w; //relative angular speed

    UGVModel::Input u = CUDA_MAT::get_control_uniform_bin_4(s_relative, *S_A, ubc);
    return make_float3(u.acc,u.alpha,0.0f);
  }

  __host__ __device__
  float3 update_site_target(const UGVModel::State &s, const float3 &site)
  {
    float3 output = site;
    output.x += s.s;
    output.y += s.theta;
    return output;
  }

  template<class Model, class Evaluator, class Swarm>
  __host__ __device__
  float simulate_evaluate(const EDTMap &map, const Evaluator &eva, Model &m, const Swarm &sw, const typename Swarm::Trace &ttr, bool &collision)
  {
    typename Model::State s = m.get_ini_state();

    float cost = 0;
    float dt = PSO::PSO_SIM_DT;
    collision = false;
    int prev_i = -1;
    float3 site_target;
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      if (i!=prev_i || prev_i == -1)
      {
        //update the site target
        site_target = update_site_target(s,ttr[i]);
      }
      prev_i = i;

      float3 u = dp_control(s, site_target);
      m.model_forward(s,u,dt);

      cost += 2*u.x*u.x + 2*u.y*u.y;
      cost += eva.process_cost(s,map,t,collision);

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
    int prev_i = -1;
    float3 site_target;
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      if (i!=prev_i || prev_i == -1)
      {
        //update the site target
        site_target = update_site_target(s,ttr[i]);
      }
      prev_i = i;

      float3 u = dp_control(s, site_target);
      m.model_forward(s,u,dt);
      traj.push_back(s);
    }
   return traj;
  }

  CUDA_MAT::Matrix<4,UGVModel::Input> *S_A;

  UniformBinCarrier ubc;
};
}
#endif // UGV_DP_CONTROL_H
