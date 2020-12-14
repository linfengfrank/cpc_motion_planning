#ifndef PSO_PLANNER_H
#define PSO_PLANNER_H

#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/pso/pso_kernels.cuh>

namespace PSO
{
template<class Model, class Controller, class Evaluator, class Swarm>
class Planner
{
public:
  Planner(int num_of_ptcls=50, int num_of_epoches=20, int num_of_episodes=4):
    m_num_of_epoches(num_of_epoches),
    m_num_of_episodes(num_of_episodes),
    m_first_time(true)
  {
    m_swarm.ptcl_size = num_of_ptcls;
  }

  ~Planner()
  {

  }

  std::vector<typename Model::State> generate_trajectory()
  {
    return m_ctrl_host.template generate_trajectory<Model,Swarm>(m_model,m_swarm,result.best_loc);
  }

  void update_best_ptcl()
  {
    // Find the best cost among the swarm
    copy_best_values<Swarm>(m_best_values,m_swarm);
    int best_idx = -1;
    cublasIsamin(m_cbls_hdl,m_swarm.ptcl_size,m_best_values,1,&best_idx);

    // Update the global best
    if(best_idx != -1)
    {
      update_glb_best<Swarm>(best_idx-1,m_swarm);
    }
  }

  void plan(const EDTMap &map)
  {
    //test_plan<N>(s,goal,m_ptcls, m_best_values, m_num_of_ptcls, &result, true,m_carrier,m_cbls_hdl);
    cublasStatus_t cbls_stt;

    for (int ctt = 0; ctt <m_num_of_episodes; ctt ++)
    {
      initialize_particles<Model, Controller, Evaluator, Swarm>(m_first_time, map, m_eva,m_model,m_ctrl_dev,m_swarm);
      update_best_ptcl();
      m_first_time = false;
      for (int i=0;i<m_num_of_epoches;i++)
      {
        float weight = 0.95-(0.95-0.4)/static_cast<float>(m_num_of_epoches)*static_cast<float>(i);
        iterate_particles<Model, Controller, Evaluator,Swarm>(weight, map, m_eva,m_model,m_ctrl_dev,m_swarm);
        update_best_ptcl();
      }
    }

    CUDA_MEMCPY_D2H(&result, m_swarm.best_ptcl,sizeof(typename Swarm::Particle));
    cudaDeviceSynchronize();
  }

  void plan_de(const EDTMap &map)
  {
    //test_plan<N>(s,goal,m_ptcls, m_best_values, m_num_of_ptcls, &result, true,m_carrier,m_cbls_hdl);
    cublasStatus_t cbls_stt;

    initialize_particles<Model, Controller, Evaluator, Swarm>(m_first_time, map, m_eva,m_model,m_ctrl_dev,m_swarm);
    m_first_time = false;
    for (int ctt = 0; ctt <m_num_of_episodes; ctt++)
    {
      for (int i=0;i<m_num_of_epoches;i++)
      {
        iterate_particles_de<Model, Controller, Evaluator,Swarm>(map, m_eva,m_model,m_ctrl_dev,m_swarm);
      }
    }
    update_best_ptcl();

    CUDA_MEMCPY_D2H(&result, m_swarm.best_ptcl, sizeof(typename Swarm::Particle));
    cudaDeviceSynchronize();
  }

  void initialize()
  {
    create_particles();
    m_ctrl_dev.load_data(m_file_location, m_factory,false);
    m_ctrl_host.load_data(m_file_location, m_factory,true);
  }

  void release()
  {
    delete_particles();
    m_ctrl_dev.release_data(m_factory,false);
    m_ctrl_host.release_data(m_factory,true);
  }

  void create_particles(typename Swarm::Particle** ptr_addr, int ptcl_num)
  {
    typename Swarm::Particle* tdata = new typename Swarm::Particle[ptcl_num];
    CUDA_ALLOC_DEV_MEM(ptr_addr,ptcl_num*sizeof(typename Swarm::Particle));
    CUDA_MEMCPY_H2D(*ptr_addr,tdata,ptcl_num*sizeof(typename Swarm::Particle));
    delete [] tdata;
  }

  void create_particles()
  {
    create_particles(&m_swarm.ptcls, m_swarm.ptcl_size);
    create_particles(&m_swarm.best_ptcl, 1);

    setup_random_states<Swarm>(m_swarm);
    CUDA_ALLOC_DEV_MEM(&m_best_values,m_swarm.ptcl_size*sizeof(float));
    cublasCreate(&m_cbls_hdl);
  }

  void delete_particles()
  {
    CUDA_FREE_DEV_MEM(m_swarm.ptcls);
    CUDA_FREE_DEV_MEM(m_swarm.best_ptcl);
    CUDA_FREE_DEV_MEM(m_best_values);
    cublasDestroy(m_cbls_hdl);
  }

public:
  CUDA_MAT::CudaMatrixFactory m_factory;
  float *m_best_values; // A fix to use cublas
  int m_num_of_epoches, m_num_of_episodes;
  cublasHandle_t m_cbls_hdl;
  typename Swarm::Particle result;
  bool m_first_time;
  Evaluator m_eva;
  Model m_model;
  Controller m_ctrl_dev;
  Controller m_ctrl_host;
  Swarm m_swarm;
  std::string m_file_location;

};

}
#endif // PSO_PLANNER_H
