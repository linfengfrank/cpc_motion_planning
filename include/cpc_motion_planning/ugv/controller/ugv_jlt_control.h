#ifndef UGV_JLT_CONTROL_H
#define UGV_JLT_CONTROL_H
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/ugv/model/ugv_model.h>
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

  void load_data(const std::string& file_location, CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {
    m_limit[1].aMax = 0.5;
    m_limit[1].aMin = -0.5;

    m_limit[1].jMax = 0.5;
    m_limit[1].jMin = -0.5;

//    m_limit[1].jMax = 0.5;
//    m_limit[1].jMin = -0.5;
  }

  void release_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_from_host)
  {

  }

  __host__ __device__
  void set_ini_state(const UGV::UGVModel::State &s)
  {
    //translation
    m_state[0].p = 0;
    m_state[0].v = s.s;
    m_state[0].a = s.v;

    //rotation
    m_state[1].p = 0;
    m_state[1].v = s.theta;
    m_state[1].a = s.w;
  }

  __host__ __device__
  void jlt_generate(const float3 &site)
  {
    //translation
    m_jlt_planner.solveVelocityTaret(site.x,m_state[0],m_limit[0],m_param[0]);

    //rotation
    m_jlt_planner.solveVelocityTaret(site.y,m_state[1],m_limit[1],m_param[1]);
  }

  __host__ __device__
  float3 update_site_target(const UGVModel::State &s, const float3 &site)
  {
    float3 output = site;
    output.x += s.s;
    output.y += s.theta;
    return output;
  }

  __host__ __device__
  void get_trajectory(UGV::UGVModel::State &s, const float &t, float2 &u)
  {
    //translation
    JLT::State tmp;
    tmp = m_jlt_planner.stageRefGen(m_param[0],t,u.x);
    s.s = tmp.v;
    s.v = tmp.a;

    //rotation
    tmp = m_jlt_planner.stageRefGen(m_param[1],t,u.y);
    s.theta = tmp.v;
    s.w = tmp.a;
  }


  template<class Model, class Evaluator, class Swarm>
  __host__ __device__
  float simulate_evaluate(const EDTMap &map, const Evaluator &eva, Model &m, const Swarm &sw, const typename Swarm::Trace &ttr, bool &collision)
  {
    typename Model::State s = m.get_ini_state();
    float cost = 0;
    float dt = PSO::PSO_SIM_DT;
    int curr_site_idx = -1;
    float start_time = 0.0f;
    float2 u;
    PSO::EvaData data;
    data.collision = false;
    int prev_i = -1;
    float3 site_target;
    for (float t=dt; t<sw.total_t; t+=dt)
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

      if (i > curr_site_idx)
      {
        curr_site_idx = i;
        start_time = t;
        set_ini_state(s);
        jlt_generate(site_target);
      }
      get_trajectory(s,t+dt-start_time,u);
      s.p.x = s.p.x + (s.v*dt )*cos(s.theta);
      s.p.y = s.p.y + (s.v*dt )*sin(s.theta);
      cost += 0.1f*s.v*s.v + 0.2f*s.w*s.w;
      cost += 0.1f*u.x*u.x + 0.4f*u.y*u.y;
      cost += eva.process_cost(s,map,t,data);

    }
    cost += eva.final_cost(s,map);
    collision = data.collision;
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
    float2 u;
    int prev_i = -1;
    float3 site_target;
    for (float t=0.0f; t<sw.total_t; t+=dt)
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

      if (i > curr_site_idx)
      {
        curr_site_idx = i;
        start_time = t;
        set_ini_state(s);
        jlt_generate(site_target);
      }

      get_trajectory(s,t+dt-start_time,u);
      s.p.x = s.p.x + (s.v*dt )*cos(s.theta);
      s.p.y = s.p.y + (s.v*dt )*sin(s.theta);
      traj.push_back(s);
    }
   return traj;
  }

  JLT m_jlt_planner;
  JLT::Limit m_limit[2];
  JLT::StageParam m_param[2];
  JLT::State m_state[2];
};
}
#endif // UGV_JLT_CONTROL_H
