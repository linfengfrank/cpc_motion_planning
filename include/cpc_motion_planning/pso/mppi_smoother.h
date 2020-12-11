#ifndef MPPI_SMOOTHER_H
#define MPPI_SMOOTHER_H
#include <cpc_motion_planning/pso/pso_kernels.cuh>
namespace PSO
{
template<class Model, class Controller, class Evaluator, class Swarm>
class MppiSmoother
{
public:
  MppiSmoother(Model *mdl, Controller *ctrl, Evaluator *eva, Swarm *sw):
    m_model(mdl),
    m_ctrl(ctrl),
    m_eva(eva),
    m_swarm(sw),
    m_N(10)
  {
    m_ptcl_group = new typename Swarm::Particle[m_swarm->ptcl_size*m_N];
    m_w_group = new float[m_swarm->ptcl_size*m_N];
  }

  ~MppiSmoother()
  {
    delete [] m_ptcl_group;
    delete [] m_w_group;
  }

public:
  typename Swarm::Trace perform_path_integral(const EDTMap &map, const typename Swarm::Trace& tr)
  {
    typename Swarm::Trace output = tr;
    int byte_size = m_swarm->ptcl_size*sizeof(typename Swarm::Particle);
    for (int i=0; i<m_N; i++)
    {
      path_integral<Model, Controller, Evaluator,Swarm>(map,*m_eva,*m_model,*m_ctrl,*m_swarm, tr);
      CUDA_MEMCPY_D2H(&m_ptcl_group[m_swarm->ptcl_size*i], m_swarm->ptcls, byte_size);
    }

    float Smin=1e6;
    for (int i=0; i<m_swarm->ptcl_size*m_N; i++)
    {
      if(m_ptcl_group[i].best_cost < Smin)
        Smin = m_ptcl_group[i].best_cost;
    }

    float eta=0;
    float lambada = 2.0f;
    for (int i=0; i<m_swarm->ptcl_size*m_N; i++)
    {
      eta += exp(-1/lambada*(m_ptcl_group[i].best_cost-Smin));
    }

    for (int i=0; i<m_swarm->ptcl_size*m_N; i++)
    {
      m_w_group[i] = 1/eta*exp(-1/lambada*(m_ptcl_group[i].best_cost-Smin));
    }

    for (int step=0; step<m_swarm->steps; step++)
    {
      for (int i=0; i<m_swarm->ptcl_size*m_N; i++)
      {
        output[step].x += m_w_group[i]*m_ptcl_group[i].ptcl_vel[step].x;
        output[step].y += m_w_group[i]*m_ptcl_group[i].ptcl_vel[step].y;
      }
    }
    std::cout<<"!!!: "<<eta<<std::endl;
    return output;
  }

private:
  Model *m_model;
  Controller *m_ctrl;
  Evaluator *m_eva;
  Swarm *m_swarm;
  typename Swarm::Particle* m_ptcl_group;
  float *m_w_group;
  int m_N;
};
}

#endif // MPPI_SMOOTHER_H
