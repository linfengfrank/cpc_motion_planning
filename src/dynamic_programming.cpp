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

  size_t M = 50;
  CUDA_MAT::Vecf pos_bins(&M);
  pos_bins.setup_device();
  float *pos_bin_data = new float[M];
  for (int i=0;i<50;i++)
  {
    pos_bin_data[i] = -10.0f + 0.4f*static_cast<float>(i);
  }
  pos_bins.upload_data(pos_bin_data);
  delete [] pos_bin_data;


  CUDA_MAT::Vecf vel_bins(&M);
  vel_bins.setup_device();
  float *vel_bin_data = new float[M];
  for (int i=0;i<50;i++)
  {
    vel_bin_data[i] = -5.0f + 0.2f*static_cast<float>(i);
  }
  vel_bins.upload_data(vel_bin_data);
  delete [] vel_bin_data;



  CUDA_MAT::Vecf theta_bins(&M);
  theta_bins.setup_device();
  float *theta_bin_data = new float[M];
  for (int i=0;i<50;i++)
  {
    theta_bin_data[i] = -3.14f + 0.13f*static_cast<float>(i);
  }
  theta_bins.upload_data(theta_bin_data);
  delete [] theta_bin_data;

  CUDA_MAT::Vecf w_bins(&M);
  w_bins.setup_device();
  float *w_bin_data = new float[M];
  for (int i=0;i<50;i++)
  {
    w_bin_data[i] = -2.5f + 0.1f*static_cast<float>(i);
  }
  w_bins.upload_data(w_bin_data);
  delete [] w_bin_data;

  size_t M_S[4] = {50,50,50,50};
  CUDA_MAT::Mat4f S_1(M_S);
  S_1.setup_device();

  CUDA_MAT::Mat4f S_2(M_S);
  S_2.setup_device();

  CUDA_MAT::Mat4Act S_A(M_S);
  S_A.setup_device();

  GPU_DP::program(S_A,S_1,S_2,pos_bins,vel_bins,theta_bins,w_bins);

  float *S = new float[50*50*50*50];
  S_1.download_data(S);

//  std::ofstream myfile;
//  myfile.open ("/home/sp/cpc_ws/test.txt");

//  for (int i = 0; i<50*50*50; i++)
//  {
//    myfile<<S[i]<<" ";
//  }

//  myfile.close();


  dp_action *SA = new dp_action[50*50*50*50];
  S_A.download_data(SA);

  // run the simulation
  std::ofstream myfile;
//  myfile.open ("/home/sp/cpc_ws/traj.txt");
//  float s[4]={-2,1,-2,1};
//  int s_idx[4];
//  int mat_idx;
//  for (int t = 0; t < 100; ++t)
//  {
//    for (int i = 0; i < 4; ++i)
//    {
//      s_idx[i] = static_cast<int>(floor(10*(s[i] + 2.5) + 0.5));
//    }
//    mat_idx = s_idx[0]*50*50*50+s_idx[1]*50*50+s_idx[2]*50+s_idx[3];
//    dp_action u = SA[mat_idx];


//    float DT = 0.05;

//    s[0] = s[0] + s[1]*DT + 0.5*u.acc*DT*DT;
//    s[1] = s[1] + u.acc*DT;
//    s[2] = s[2] + s[3]*DT + 0.5*u.alpha*DT*DT;
//    s[3] = s[3] + u.alpha*DT;


////    s[0] = s[0] + s[1]*0.05 + 0.5*u.jerk*0.05*0.05;
////    s[1] = s[1] + u.jerk*0.05;
////    s[2] = s[2] + u.alpha*0.05;

//    myfile<<s[0]<<" "<<s[1]<<" "<<s[2]<<" "<<s[3]<<std::endl;

//  }
//  myfile.close();

  myfile.open ("/home/sp/cpc_ws/acc.txt");

  for (int i = 0; i<50*50*50*50; i++)
  {
    myfile<<SA[i].acc<<" ";
  }

  myfile.close();

  myfile.open ("/home/sp/cpc_ws/alpha.txt");

  for (int i = 0; i<50*50*50*50; i++)
  {
    myfile<<SA[i].alpha<<" ";
  }

  myfile.close();






  vel_bins.free_device();
  pos_bins.free_device();
  w_bins.free_device();
  S_1.free_device();
  S_2.free_device();
  delete [] S;
  delete [] SA;


  std::cout<<"Finish the dynamic programming"<<std::endl;
  return 0;
}
