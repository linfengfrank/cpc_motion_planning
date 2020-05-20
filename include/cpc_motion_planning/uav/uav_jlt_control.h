#ifndef UAV_JLT_CONTROL_H
#define UAV_JLT_CONTROL_H
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/uav/uav_model.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/JLT.h>

namespace UAV
{
class UAVJLTControl
{
public:
  UAVJLTControl()
  {

  }

  ~UAVJLTControl()
  {

  }

  void load_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {
    //    m_limit[1].aMax = 0.5;
    //    m_limit[1].aMin = -0.5;

    //    m_limit[1].jMax = 0.5;
    //    m_limit[1].jMin = -0.5;

    //    m_limit[1].jMax = 0.5;
    //    m_limit[1].jMin = -0.5;
  }

  void release_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_from_host)
  {

  }

  __host__ __device__
  void set_ini_state(const UAV::UAVModel::State &s)
  {
    //x
    m_state[0].p = s.p.x;
    m_state[0].v = s.v.x;
    m_state[0].a = s.a.x;

    //y
    m_state[1].p = s.p.y;
    m_state[1].v = s.v.y;
    m_state[1].a = s.a.y;

    //z
    m_state[2].p = s.p.z;
    m_state[2].v = s.v.z;
    m_state[2].a = s.a.z;
  }

  __host__ __device__
  void jlt_generate(const float3 &site)
  {
    //x
    m_jlt_planner.solveTPBVP(site.x,0,m_state[0],m_limit[0],m_param[0]);

    //y
    m_jlt_planner.solveTPBVP(site.y,0,m_state[1],m_limit[0],m_param[1]);

    //z
    m_jlt_planner.solveTPBVP(site.z,0,m_state[2],m_limit[1],m_param[2]);
  }

  __host__ __device__
  void get_trajectory(UAV::UAVModel::State &s, const float &t, float3 &u)
  {
    //x
    JLT::State tmp;
    tmp = m_jlt_planner.TPBVPRefGen(m_param[0], t, u.x);
    s.p.x = tmp.p;
    s.v.x = tmp.v;
    s.a.x = tmp.a;

    //x
    tmp = m_jlt_planner.TPBVPRefGen(m_param[1], t, u.y);
    s.p.y = tmp.p;
    s.v.y = tmp.v;
    s.a.y = tmp.a;
    //x
    tmp = m_jlt_planner.TPBVPRefGen(m_param[2], t, u.z);
    s.p.z = tmp.p;
    s.v.z = tmp.v;
    s.a.z = tmp.a;
  }


  template<class Model, class Evaluator, class Swarm>
  __host__ __device__
  float simulate_evaluate(const EDTMap &map, const Evaluator &eva, Model &m, const Swarm &sw, const typename Swarm::Trace &ttr)
  {
    typename Model::State s = m.get_ini_state();
    float cost = 0;
    float dt = PSO::PSO_SIM_DT;
    int curr_site_idx = -1;
    float start_time = 0.0f;
    float3 u;
    for (float t=dt; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      if (i > curr_site_idx)
      {
        curr_site_idx = i;
        start_time = t;
        set_ini_state(s);
        jlt_generate(ttr[curr_site_idx]);
      }
      get_trajectory(s,t+dt-start_time,u);


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

    int curr_site_idx = -1;
    float start_time = 0.0f;
    float dt = PSO::PSO_CTRL_DT;
    float3 u;
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      int i = static_cast<int>(floor(t/sw.step_dt));
      if (i > sw.steps - 1)
        i = sw.steps - 1;

      if (i > curr_site_idx)
      {
        curr_site_idx = i;
        start_time = t;
        set_ini_state(s);
        jlt_generate(ttr[curr_site_idx]);
      }

      get_trajectory(s,t+dt-start_time,u);
      traj.push_back(s);
    }
    return traj;
  }

  JLT m_jlt_planner;
  JLT::Limit m_limit[2];
  JLT::TPBVPParam m_param[3];
  JLT::State m_state[3];
};
}
#endif // UAV_JLT_CONTROL_H
