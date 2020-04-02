#ifndef PSO_PLANNER_H
#define PSO_PLANNER_H
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/pso/pso_kernels.cuh>

namespace PSO
{
template <int N>
class Planner
{
public:
  Planner(int num_of_ptcls=50, int num_of_epoches=20):
    m_num_of_ptcls(num_of_ptcls),
    m_num_of_epoches(num_of_epoches)
  {
  }

  ~Planner()
  {

  }

  void plan(const State &s, const State &goal)
  {
    //test_plan<N>(s,goal,m_ptcls, m_best_values, m_num_of_ptcls, &result, true,m_carrier,m_cbls_hdl);
    cublasStatus_t cbls_stt;

    initialize_particles(m_ptcls,m_num_of_ptcls,true,s,goal,m_carrier);
    for (int i=0;i<m_num_of_epoches;i++)
    {
      iterate_particles(m_ptcls,m_num_of_ptcls,1.0f,s,goal,m_carrier);
      copy_best_values(m_ptcls,m_num_of_ptcls,m_best_values);

      int best_idx = -1;
      cbls_stt = cublasIsamin(m_cbls_hdl,m_num_of_ptcls,m_best_values,1,&best_idx);

      if(best_idx != -1)
      {
        CUDA_MEMCPY_D2D(m_ptcls+m_num_of_ptcls-1,m_ptcls+best_idx-1,sizeof(Particle));
      }
    }

    CUDA_MEMCPY_D2H(&result, m_ptcls+m_num_of_ptcls-1,sizeof(Particle));
    cudaDeviceSynchronize();
  }

  void create_particles()
  {
    Particle* data = new Particle[m_num_of_ptcls];
    CUDA_ALLOC_DEV_MEM(&m_ptcls,m_num_of_ptcls*sizeof(Particle));
    CUDA_MEMCPY_H2D(m_ptcls,data,m_num_of_ptcls*sizeof(Particle));
    delete [] data;
    setup_random_states(m_ptcls, m_num_of_ptcls);

    CUDA_ALLOC_DEV_MEM(&m_best_values,m_num_of_ptcls*sizeof(float));
    cublasCreate(&m_cbls_hdl);
  }

  void delete_particles()
  {
    CUDA_FREE_DEV_MEM(m_ptcls);
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
  VoidPtrCarrier<N> m_carrier;
  Particle *m_ptcls;
  float *m_best_values; // A fix to use cublas
  int m_num_of_ptcls;
  int m_num_of_epoches;
  cublasHandle_t m_cbls_hdl;
  Particle result;

};

}
#endif // PSO_PLANNER_H
