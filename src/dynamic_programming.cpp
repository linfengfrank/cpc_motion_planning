#include <ros/ros.h>
#include <iostream>
#include <cuda_geometry/cuda_geometry.cuh>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/dynamic_programming.h>
void test(int g[3])
{
  std::cout<<g[0]+g[1]+g[2]<<std::endl;
}

int main(int argc, char **argv)
{
  std::cout<<"Start the dynamic programming"<<std::endl;

  std::cout<<"Allocating device memory"<<std::endl;

  size_t M = 50;
  CUDA_MAT::Vecf vel_bins(&M);
  vel_bins.setup_device();

  float *vel_bin_data = new float[M];

  for (int i=0;i<50;i++)
  {
    vel_bin_data[i] = -2.0f + 0.15f*static_cast<float>(i);
  }

  vel_bins.upload_data(vel_bin_data);
  delete [] vel_bin_data;

  GPU_DP::program(vel_bins);
  vel_bins.free_device();
//  float *states;
//  CUDA_ALLOC_DEV_MEM(&states,1000);

//  CUDA_FREE_DEV_MEM(states);
//  int a[3] = {1, 2, 3};
//  test(a);

  std::cout<<"Finish the dynamic programming"<<std::endl;
  return 0;
}
