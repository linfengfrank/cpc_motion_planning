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
  Planner(int num_of_ptcls)
  {
    m_num_of_ptcls = num_of_ptcls;
  }

  ~Planner()
  {

  }

  void create_particles()
  {
    Particle* data = new Particle[m_num_of_ptcls];
    m_ptcls = m_factory.make_cuda_matrix<1,Particle>(&m_num_of_ptcls, data);
    delete [] data;

    setup_random_states(m_ptcls, m_num_of_ptcls);
  }

  void delete_particles()
  {
    m_factory.free_cuda_matrix<1,Particle>(m_ptcls);
  }

  void load_data_matrix()
  {
    m_carrier[0] = m_factory.load_cuda_matrix<4,dp_action>("/home/sp/cpc_ws/SA.dat");
    m_carrier[1] = m_factory.load_cuda_matrix<1,float>("/home/sp/cpc_ws/pos_bin.dat");
    m_carrier[2] = m_factory.load_cuda_matrix<1,float>("/home/sp/cpc_ws/vel_bin.dat");
    m_carrier[3] = m_factory.load_cuda_matrix<1,float>("/home/sp/cpc_ws/theta_bin.dat");
    m_carrier[4] = m_factory.load_cuda_matrix<1,float>("/home/sp/cpc_ws/w_bin.dat");
  }

  void free_data_matrix()
  {
    m_factory.free_cuda_matrix<4,dp_action>(m_carrier[0]);
    m_factory.free_cuda_matrix<1,float>(m_carrier[1]);
    m_factory.free_cuda_matrix<1,float>(m_carrier[2]);
    m_factory.free_cuda_matrix<1,float>(m_carrier[3]);
    m_factory.free_cuda_matrix<1,float>(m_carrier[4]);
  }

private:
  CUDA_MAT::CudaMatrixFactory m_factory;
  VoidPtrCarrier<5> m_carrier;
  void *m_ptcls;
  size_t m_num_of_ptcls;

};

}
#endif // PSO_PLANNER_H
