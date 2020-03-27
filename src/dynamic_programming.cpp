#include <ros/ros.h>
#include <iostream>
#include <cuda_geometry/cuda_geometry.cuh>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/dynamic_programming.h>
#include <fstream>

int main(int argc, char **argv)
{
  std::cout<<"Start the dynamic programming"<<std::endl;

  std::cout<<"Allocating device memory"<<std::endl;
  //------
  size_t M_pos = 60;
  CUDA_MAT::Vecf pos_bins(&M_pos);
  pos_bins.setup_device();
  float *pos_bin_data = new float[M_pos];
  for (int i=0;i<M_pos;i++)
  {
    pos_bin_data[i] = GPU_DP::pos_gen_val(i);
  }
  pos_bins.upload_data(pos_bin_data);
  delete [] pos_bin_data;
  pos_bins.write_to_file("/home/sp/cpc_ws/pos_bin.dat");

  //------
  size_t M_vel = 40;
  CUDA_MAT::Vecf vel_bins(&M_vel);
  vel_bins.setup_device();
  float *vel_bin_data = new float[M_vel];
  for (int i=0;i<M_vel;i++)
  {
    vel_bin_data[i] = GPU_DP::vel_gen_val(i);
  }
  vel_bins.upload_data(vel_bin_data);
  delete [] vel_bin_data;
  vel_bins.write_to_file ("/home/sp/cpc_ws/vel_bin.dat");

  //------
  size_t M_theta = 50;
  CUDA_MAT::Vecf theta_bins(&M_theta);
  theta_bins.setup_device();
  float *theta_bin_data = new float[M_theta];
  for (int i=0;i<M_theta;i++)
  {
    theta_bin_data[i] = GPU_DP::theta_gen_val(i);
  }
  theta_bins.upload_data(theta_bin_data);
  delete [] theta_bin_data;
  theta_bins.write_to_file ("/home/sp/cpc_ws/theta_bin.dat");

  //------
  size_t M_w = 50;
  CUDA_MAT::Vecf w_bins(&M_w);
  w_bins.setup_device();
  float *w_bin_data = new float[M_w];
  for (int i=0;i<M_w;i++)
  {
    w_bin_data[i] = GPU_DP::w_gen_val(i);
  }
  w_bins.upload_data(w_bin_data);
  delete [] w_bin_data;
  w_bins.write_to_file ("/home/sp/cpc_ws/w_bin.dat");

  //------
  size_t M_S[4] = {M_pos,M_vel,M_theta,M_w};
  CUDA_MAT::Mat4f S_1(M_S);
  S_1.setup_device();

  CUDA_MAT::Mat4f S_2(M_S);
  S_2.setup_device();

  CUDA_MAT::Mat4Act S_A(M_S);
  S_A.setup_device();

  //------
  std::cout<<"Start dynamic programming"<<std::endl;
  GPU_DP::program(S_A,S_1,S_2,pos_bins,vel_bins,theta_bins,w_bins);

  //------
  std::cout<<"Write data to files"<<std::endl;
  S_A.write_to_file ("/home/sp/cpc_ws/SA.dat");

  std::cout<<"Release the resources"<<std::endl;
  vel_bins.free_device();
  pos_bins.free_device();
  w_bins.free_device();
  theta_bins.free_device();
  S_1.free_device();
  S_2.free_device();
  S_A.free_device();

  std::cout<<"Finish the dynamic programming"<<std::endl;
  return 0;
}
