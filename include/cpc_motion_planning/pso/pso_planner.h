#ifndef PSO_PLANNER_H
#define PSO_PLANNER_H
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/pso/pso_kernels.cuh>

namespace PSO
{
class Planner
{
public:
  Planner(int num_of_ptcls=50, int num_of_epoches=20):
    m_num_of_epoches(num_of_epoches),
    m_first_time(true)
  {
    m_swam.ptcl_size = num_of_ptcls;
  }

  ~Planner()
  {

  }

  void plan(const State &s, const State &goal, const EDTMap &map)
  {
    //test_plan<N>(s,goal,m_ptcls, m_best_values, m_num_of_ptcls, &result, true,m_carrier,m_cbls_hdl);
    cublasStatus_t cbls_stt;

    initialize_particles(m_swam, m_first_time, s, goal, m_carrier, m_ubc, map, result.best_loc);
    m_first_time = false;
    for (int i=0;i<m_num_of_epoches;i++)
    {
      float weight = 0.95-(0.95-0.4)/static_cast<float>(m_num_of_epoches)*static_cast<float>(i);
      iterate_particles(m_swam, weight, s, goal, m_carrier, m_ubc, map, result.best_loc);
      copy_best_values(m_swam,m_best_values);

      int best_idx = -1;
      cbls_stt = cublasIsamin(m_cbls_hdl,m_swam.ptcl_size,m_best_values,1,&best_idx);

      if(best_idx != -1)
      {
        CUDA_MEMCPY_D2D(m_swam.ptcls+m_swam.ptcl_size-1,m_swam.ptcls+best_idx-1,sizeof(Particle));
      }
    }

    CUDA_MEMCPY_D2H(&result, m_swam.ptcls+m_swam.ptcl_size-1,sizeof(Particle));
    cudaDeviceSynchronize();
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

  void load_data_matrix(bool load_to_host = false)
  {
    m_carrier[0] = m_factory.load_cuda_matrix<4,dp_action>("/home/sp/cpc_ws/SA.dat",load_to_host);
    m_carrier[1] = m_factory.load_cuda_matrix<1,float>("/home/sp/cpc_ws/pos_bin.dat",load_to_host);
    m_carrier[2] = m_factory.load_cuda_matrix<1,float>("/home/sp/cpc_ws/vel_bin.dat",load_to_host);
    m_carrier[3] = m_factory.load_cuda_matrix<1,float>("/home/sp/cpc_ws/theta_bin.dat",load_to_host);
    m_carrier[4] = m_factory.load_cuda_matrix<1,float>("/home/sp/cpc_ws/w_bin.dat",load_to_host);

    m_factory.load_uniform_bin("/home/sp/cpc_ws/pos_bin.dat",m_ubc.bins[0]);
    m_factory.load_uniform_bin("/home/sp/cpc_ws/vel_bin.dat",m_ubc.bins[1]);
    m_factory.load_uniform_bin("/home/sp/cpc_ws/theta_bin.dat",m_ubc.bins[2]);
    m_factory.load_uniform_bin("/home/sp/cpc_ws/w_bin.dat",m_ubc.bins[3]);
  }

  void free_data_matrix(bool load_from_host = false)
  {
    m_factory.free_cuda_matrix<4,dp_action>(m_carrier[0], load_from_host);
    m_factory.free_cuda_matrix<1,float>(m_carrier[1], load_from_host);
    m_factory.free_cuda_matrix<1,float>(m_carrier[2], load_from_host);
    m_factory.free_cuda_matrix<1,float>(m_carrier[3], load_from_host);
    m_factory.free_cuda_matrix<1,float>(m_carrier[4], load_from_host);
  }

public:
  CUDA_MAT::CudaMatrixFactory m_factory;
  VoidPtrCarrier m_carrier;
  Swarm m_swam;
  //Particle *m_ptcls;
  float *m_best_values; // A fix to use cublas
  //int m_num_of_ptcls;
  int m_num_of_epoches;
  cublasHandle_t m_cbls_hdl;
  Particle result;
  bool m_first_time;
  UniformBinCarrier m_ubc;

};

}
#endif // PSO_PLANNER_H
