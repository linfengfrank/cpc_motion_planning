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
  UAVJLTControl():m_theta(0)
  {

  }

  ~UAVJLTControl()
  {

  }

  void load_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_to_host)
  {
        m_limit[0].aMax = 2.5;
        m_limit[0].aMin = -2.5;

        m_limit[0].jMax = 3.5;
        m_limit[0].jMin = -3.5;

        m_limit[0].vMax = 4.5;
        m_limit[0].vMin = -4.5;
  }

  void release_data(CUDA_MAT::CudaMatrixFactory &factory, bool load_from_host)
  {

  }

  __host__ __device__
  UAV::UAVModel::State transform(const UAV::UAVModel::State &in, const float3 &cnt)
  {
    UAV::UAVModel::State out;

    float cos_t = cosf(cnt.z);
    float sin_t = sinf(cnt.z);
    float dx = in.p.x - cnt.x;
    float dy = in.p.y - cnt.y;

    out.p.x = dx*cos_t + dy*sin_t;
    out.p.y = -dx*sin_t + dy*cos_t;
    out.p.z = in.p.z;

    out.v.x = in.v.x*cos_t + in.v.y*sin_t;
    out.v.y = -in.v.x*sin_t + in.v.y*cos_t;
    out.v.z = in.v.z;

    out.a.x = in.a.x*cos_t + in.a.y*sin_t;
    out.a.y = -in.a.x*sin_t + in.a.y*cos_t;
    out.a.z = in.a.z;

    return out;
  }

  __host__ __device__
  float3 transform(const float3 &in, const float3 &cnt)
  {
    float3 out;

    float cos_t = cosf(cnt.z);
    float sin_t = sinf(cnt.z);
    float dx = in.x - cnt.x;
    float dy = in.y - cnt.y;

    out.x = dx*cos_t + dy*sin_t;
    out.y = -dx*sin_t + dy*cos_t;
    out.z = in.z;

    return out;
  }

  __host__ __device__
  UAV::UAVModel::State transform_back(const UAV::UAVModel::State &in, const float3 &cnt)
  {
    UAV::UAVModel::State out;

    float cos_t = cosf(cnt.z);
    float sin_t = sinf(cnt.z);

    out.p.x = in.p.x*cos_t - in.p.y*sin_t + cnt.x;
    out.p.y = in.p.x*sin_t + in.p.y*cos_t + cnt.y;
    out.p.z = in.p.z;

    out.v.x = in.v.x*cos_t - in.v.y*sin_t;
    out.v.y = in.v.x*sin_t + in.v.y*cos_t;
    out.v.z = in.v.z;

    out.a.x = in.a.x*cos_t - in.a.y*sin_t;
    out.a.y = in.a.x*sin_t + in.a.y*cos_t;
    out.a.z = in.a.z;

    return out;
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
    float3 trans = make_float3(s.p.x,s.p.y,m_theta);
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
        set_ini_state(transform(s,trans));
        jlt_generate(transform(ttr[curr_site_idx],trans));
      }
      get_trajectory(s,t+dt-start_time,u);


      cost += eva.process_cost(transform_back(s,trans),map);

    }
    cost += eva.final_cost(transform_back(s,trans),map);

    return cost;
  }

  template<class Model, class Swarm>
  __host__ __device__
  std::vector<typename Model::State> generate_trajectory(Model &m, const Swarm &sw, const typename Swarm::Trace &ttr)
  {
    std::vector<typename Model::State> traj;
    typename Model::State s = m.get_ini_state();
    float3 trans = make_float3(s.p.x,s.p.y,m_theta);
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
        set_ini_state(transform(s,trans));
        jlt_generate(transform(ttr[curr_site_idx],trans));
      }

      get_trajectory(s,t+dt-start_time,u);
      traj.push_back(transform_back(s,trans));
    }
    return traj;
  }

  JLT m_jlt_planner;
  JLT::Limit m_limit[2];
  JLT::TPBVPParam m_param[3];
  JLT::State m_state[3];
  float m_theta;
};
}
#endif // UAV_JLT_CONTROL_H
