#include <ros/ros.h>
#include <iostream>
#include <cuda_geometry/cuda_geometry.cuh>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/dynamic_programming.h>
#include <fstream>

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
    vel_bin_data[i] = -2.5f + 0.1f*static_cast<float>(i);
  }
  vel_bins.upload_data(vel_bin_data);
  delete [] vel_bin_data;

  CUDA_MAT::Vecf acc_bins(&M);
  acc_bins.setup_device();
  float *acc_bin_data = new float[M];
  for (int i=0;i<50;i++)
  {
    acc_bin_data[i] = -2.5f + 0.1f*static_cast<float>(i);
  }
  acc_bins.upload_data(acc_bin_data);
  delete [] acc_bin_data;


  CUDA_MAT::Vecf w_bins(&M);
  w_bins.setup_device();
  float *w_bin_data = new float[M];
  for (int i=0;i<50;i++)
  {
    w_bin_data[i] = -2.5f + 0.1f*static_cast<float>(i);
  }
  w_bins.upload_data(w_bin_data);
  delete [] w_bin_data;

  size_t M_S[3] = {50,50,50};
  CUDA_MAT::Mat3f S_1(M_S);
  S_1.setup_device();

  CUDA_MAT::Mat3f S_2(M_S);
  S_2.setup_device();

  CUDA_MAT::Mat3Act S_A(M_S);
  S_A.setup_device();

  GPU_DP::program(S_A,S_1,S_2,vel_bins,acc_bins,w_bins);

  float *S = new float[50*50*50];
  S_1.download_data(S);

//  std::ofstream myfile;
//  myfile.open ("/home/sp/cpc_ws/test.txt");

//  for (int i = 0; i<50*50*50; i++)
//  {
//    myfile<<S[i]<<" ";
//  }

//  myfile.close();


  dp_action *SA = new dp_action[50*50*50];
  S_A.download_data(SA);

  // run the simulation
  std::ofstream myfile;
  myfile.open ("/home/sp/cpc_ws/traj.txt");
  float s[3]={-2,1,-2};
  int s_idx[3];
  int mat_idx;
  for (int t = 0; t < 100; ++t)
  {
    for (int i = 0; i < 3; ++i)
    {
      s_idx[i] = static_cast<int>(floor(10*(s[i] + 2.5) + 0.5));
    }
    mat_idx = s_idx[0]*2500+s_idx[1]*50+s_idx[2];
    dp_action u = SA[mat_idx];


    s[0] = s[0] + s[1]*0.05 + 0.5*u.jerk*0.05*0.05;
    s[1] = s[1] + u.jerk*0.05;
    s[2] = s[2] + u.alpha*0.05;

    myfile<<s[0]<<" "<<s[1]<<" "<<s[2]<<std::endl;

  }
  myfile.close();

//  myfile.open ("/home/sp/cpc_ws/jerk.txt");

//  for (int i = 0; i<50*50*50; i++)
//  {
//    myfile<<SA[i].jerk<<" ";
//  }

//  myfile.close();

//  myfile.open ("/home/sp/cpc_ws/alpha.txt");

//  for (int i = 0; i<50*50*50; i++)
//  {
//    myfile<<SA[i].alpha<<" ";
//  }

//  myfile.close();






  vel_bins.free_device();
  acc_bins.free_device();
  w_bins.free_device();
  S_1.free_device();
  S_2.free_device();
  delete [] S;
  delete [] SA;


  std::cout<<"Finish the dynamic programming"<<std::endl;
  return 0;
}
