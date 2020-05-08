#ifndef PSO_PLANNER_H
#define PSO_PLANNER_H
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/pso/pso_kernels.cuh>
#include <cpc_motion_planning/pso/single_target_evluator.h>
namespace PSO
{
template<class Model, class Controler, class Evaluator>
class Planner
{
public:
  Planner(int num_of_ptcls=50, int num_of_epoches=20, int num_of_episodes=4):
    m_num_of_epoches(num_of_epoches),
    m_num_of_episodes(num_of_episodes),
    m_first_time(true),
    m_on_host(false)
  {
    m_swam.ptcl_size = num_of_ptcls;
  }

  ~Planner()
  {

  }

  void plan(const UAVModel::State &s, const SingleTargetEvaluator::Target &goal, const EDTMap &map)
  {
    //test_plan<N>(s,goal,m_ptcls, m_best_values, m_num_of_ptcls, &result, true,m_carrier,m_cbls_hdl);
    cublasStatus_t cbls_stt;

    m_eva.setTarget(goal);
    m_model.set_ini_state(s);

    for (int ctt = 0; ctt <m_num_of_episodes; ctt ++)
    {
      initialize_particles<Model, Controler, Evaluator>(m_swam, m_first_time, map, result.best_loc,m_eva,m_model,m_dp_ctrl);
      m_first_time = false;
      for (int i=0;i<m_num_of_epoches;i++)
      {
        float weight = 0.95-(0.95-0.4)/static_cast<float>(m_num_of_epoches)*static_cast<float>(i);
        iterate_particles<Model, Controler, Evaluator>(m_swam, weight, map, result.best_loc,m_eva,m_model,m_dp_ctrl);
        copy_best_values(m_swam,m_best_values);

        int best_idx = -1;
        cbls_stt = cublasIsamin(m_cbls_hdl,m_swam.ptcl_size,m_best_values,1,&best_idx);

        if(best_idx != -1)
        {
          CUDA_MEMCPY_D2D(m_swam.ptcls+m_swam.ptcl_size-1,m_swam.ptcls+best_idx-1,sizeof(Particle));
        }
      }
    }

    CUDA_MEMCPY_D2H(&result, m_swam.ptcls+m_swam.ptcl_size-1,sizeof(Particle));
    cudaDeviceSynchronize();
  }

  void initialize(bool on_host = false)
  {
    m_on_host = on_host;
    if (!m_on_host)
    {
      create_particles();
    }
    m_dp_ctrl.load_data(m_factory,m_on_host);
  }

  void release()
  {
    if (!m_on_host)
    {
      delete_particles();
    }
    m_dp_ctrl.release_data(m_factory,m_on_host);
  }

  void create_particles()
  {
    Particle* data = new Particle[m_swam.ptcl_size];
    CUDA_ALLOC_DEV_MEM(&m_swam.ptcls,m_swam.ptcl_size*sizeof(Particle));
    CUDA_MEMCPY_H2D(m_swam.ptcls,data,m_swam.ptcl_size*sizeof(Particle));
    delete [] data;
    setup_random_states(m_swam);

    CUDA_ALLOC_DEV_MEM(&m_best_values,m_swam.ptcl_size*sizeof(float));
    cublasCreate(&m_cbls_hdl);
  }

  void delete_particles()
  {
    CUDA_FREE_DEV_MEM(m_swam.ptcls);
    CUDA_FREE_DEV_MEM(m_best_values);
    cublasDestroy(m_cbls_hdl);
  }

public:
  Swarm m_swam;
  CUDA_MAT::CudaMatrixFactory m_factory;
  float *m_best_values; // A fix to use cublas
  int m_num_of_epoches, m_num_of_episodes;
  cublasHandle_t m_cbls_hdl;
  Particle result;
  bool m_first_time;
  Evaluator m_eva;
  Model m_model;
  Controler m_dp_ctrl;
  bool m_on_host;

};

}
#endif // PSO_PLANNER_H
