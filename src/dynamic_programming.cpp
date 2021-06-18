#include <ros/ros.h>
#include <iostream>
#include <cuda_geometry/cuda_geometry.cuh>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <fstream>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <chrono>
#include <cpc_motion_planning/ugv/model/ugv_model.h>

int main(int argc, char **argv)
{
  std::cout<<"Start the dynamic programming"<<std::endl;

  CUDA_MAT::CudaMatrixFactory factory;

  std::cout<<"Create bins"<<std::endl;
  size_t M_pos = 100;
  float *pos_bin_data = new float[M_pos];
  for (int i=0;i<M_pos;i++)
  {
    pos_bin_data[i] = UGV::pos_gen_val(i);
  }

  //------
  size_t M_vel = 50;
  float *vel_bin_data = new float[M_vel];
  for (int i=0;i<M_vel;i++)
  {
    vel_bin_data[i] = UGV::vel_gen_val(i);
  }

  //------
  size_t M_theta = 50;
  float *theta_bin_data = new float[M_theta];
  for (int i=0;i<M_theta;i++)
  {
    theta_bin_data[i] = UGV::theta_gen_val(i);
  }

  //------
  size_t M_w = 50;
  float *w_bin_data = new float[M_w];
  for (int i=0;i<M_w;i++)
  {
    w_bin_data[i] = UGV::w_gen_val(i);
  }

  std::cout<<"Allocating device memory"<<std::endl;
  //------
  void * pos_bins, * vel_bins, * theta_bins, * w_bins;
  factory.make_cuda_matrix<1,float>(&pos_bins, &M_pos,pos_bin_data);
  factory.make_cuda_matrix<1,float>(&vel_bins, &M_vel,vel_bin_data);
  factory.make_cuda_matrix<1,float>(&theta_bins, &M_theta,theta_bin_data);
  factory.make_cuda_matrix<1,float>(&w_bins, &M_w,w_bin_data);

  //------
  size_t M_S[4] = {M_pos,M_vel,M_theta,M_w};

  void * S_1, * S_2, * S_A;
  factory.make_cuda_matrix<4,float>(&S_1, M_S);
  factory.make_cuda_matrix<4,float>(&S_2, M_S);
  factory.make_cuda_matrix<4,UGV::UGVModel::Input>(&S_A, M_S);

  //------
  std::cout<<"Start dynamic programming"<<std::endl;
  VoidPtrCarrier ptr_car;
  ptr_car[0] = S_A;
  ptr_car[1] = S_1;
  ptr_car[2] = S_2;
  ptr_car[3] = pos_bins;
  ptr_car[4] = vel_bins;
  ptr_car[5] = theta_bins;
  ptr_car[6] = w_bins;

  auto start = std::chrono::steady_clock::now();
  GPU_DP::program<UGV::UGVModel::Input>(ptr_car, M_S);
  auto end = std::chrono::steady_clock::now();
  std::cout << "Consumed: "
            << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
            << " s" << std::endl;

  //------
  std::cout<<"Write data to files"<<std::endl;

  factory.write_cuda_matrix<4,UGV::UGVModel::Input>(S_A,"/home/sp/cpc_ws/SA.dat");
  factory.write_cuda_matrix<1,float>(pos_bins,"/home/sp/cpc_ws/pos_bin.dat");
  factory.write_cuda_matrix<1,float>(vel_bins,"/home/sp/cpc_ws/vel_bin.dat");
  factory.write_cuda_matrix<1,float>(theta_bins,"/home/sp/cpc_ws/theta_bin.dat");
  factory.write_cuda_matrix<1,float>(w_bins,"/home/sp/cpc_ws/w_bin.dat");

  std::cout<<"Release the resources"<<std::endl;
  factory.free_cuda_matrix<1,float>(pos_bins);
  factory.free_cuda_matrix<1,float>(vel_bins);
  factory.free_cuda_matrix<1,float>(theta_bins);
  factory.free_cuda_matrix<1,float>(w_bins);
  factory.free_cuda_matrix<4,float>(S_1);
  factory.free_cuda_matrix<4,float>(S_2);
  factory.free_cuda_matrix<4,UGV::UGVModel::Input>(S_A);

  delete [] pos_bin_data;
  delete [] vel_bin_data;
  delete [] theta_bin_data;
  delete [] w_bin_data;

  std::cout<<"Finish the dynamic programming"<<std::endl;
  return 0;
}
