#include <ros/ros.h>
#include <iostream>
#include <cuda_geometry/cuda_geometry.cuh>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <fstream>
#include <cpc_motion_planning/cuda_matrix_factory.h>
#include <chrono>

void position_program()
{
  CUDA_MAT::CudaMatrixFactory factory;

  std::cout<<"Create bins"<<std::endl;
  size_t M_pos = 100;
  float *pos_bin_data = new float[M_pos];
  for (int i=0;i<M_pos;i++)
  {
    pos_bin_data[i] = UAV::pos_gen_val(i);
  }

  //------
  size_t M_vel = 50;
  float *vel_bin_data = new float[M_vel];
  for (int i=0;i<M_vel;i++)
  {
    vel_bin_data[i] = UAV::vel_gen_val(i);
  }

  //------
  size_t M_acc = 50;
  float *acc_bin_data = new float[M_acc];
  for (int i=0;i<M_acc;i++)
  {
    acc_bin_data[i] = UAV::acc_gen_val(i);
  }

  std::cout<<"Allocating device memory"<<std::endl;
  //------
  void * pos_bins = factory.make_cuda_matrix<1,float>(&M_pos,pos_bin_data);
  void * vel_bins = factory.make_cuda_matrix<1,float>(&M_vel,vel_bin_data);
  void * acc_bins = factory.make_cuda_matrix<1,float>(&M_acc,acc_bin_data);

  //------
  size_t M_S[3] = {M_pos,M_vel,M_acc};

  void * S_1 = factory.make_cuda_matrix<3,float>(M_S);
  void * S_2 = factory.make_cuda_matrix<3,float>(M_S);
  void * S_A = factory.make_cuda_matrix<3,UAV::UAVModel::Input>(M_S);

  //------
  std::cout<<"Start dynamic programming"<<std::endl;
  VoidPtrCarrier ptr_car;
  ptr_car[0] = S_A;
  ptr_car[1] = S_1;
  ptr_car[2] = S_2;
  ptr_car[3] = pos_bins;
  ptr_car[4] = vel_bins;
  ptr_car[5] = acc_bins;

  auto start = std::chrono::steady_clock::now();
  GPU_DP::program<UAV::UAVModel::Input>(ptr_car, M_S);
  auto end = std::chrono::steady_clock::now();
  std::cout << "Consumed: "
            << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
            << " s" << std::endl;

  //------
  std::cout<<"Write data to files"<<std::endl;

 // factory.write_cuda_matrix<3,UAV::UAVModel::Input>(S_A,"/home/sp/cpc_ws/SA.dat");
  //factory.write_cuda_matrix<1,float>(pos_bins,"/home/sp/cpc_ws/pos_bin.dat");
  //factory.write_cuda_matrix<1,float>(vel_bins,"/home/sp/cpc_ws/vel_bin.dat");
  //factory.write_cuda_matrix<1,float>(acc_bins,"/home/sp/cpc_ws/acc_bin.dat");
  factory.write_cuda_matrix<3,UAV::UAVModel::Input>(S_A,"/home/uas/yzchen_ws/SA.dat");
  factory.write_cuda_matrix<1,float>(pos_bins,"/home/uas/yzchen_ws/pos_bin.dat");
  factory.write_cuda_matrix<1,float>(vel_bins,"/home/uas/yzchen_ws/vel_bin.dat");
  factory.write_cuda_matrix<1,float>(acc_bins,"/home/uas/yzchen_ws/acc_bin.dat");

  std::cout<<"Release the resources"<<std::endl;
  factory.free_cuda_matrix<1,float>(pos_bins);
  factory.free_cuda_matrix<1,float>(vel_bins);
  factory.free_cuda_matrix<1,float>(acc_bins);
  factory.free_cuda_matrix<3,float>(S_1);
  factory.free_cuda_matrix<3,float>(S_2);
  factory.free_cuda_matrix<3,UAV::UAVModel::Input>(S_A);

  delete [] pos_bin_data;
  delete [] vel_bin_data;
  delete [] acc_bin_data;

}
//-----------
void velocity_program()
{
  CUDA_MAT::CudaMatrixFactory factory;

  std::cout<<"Create bins"<<std::endl;

  //------
  size_t M_vel = 50;
  float *vel_bin_data = new float[M_vel];
  for (int i=0;i<M_vel;i++)
  {
    vel_bin_data[i] = UAV::vel_gen_val(i);
  }

  //------
  size_t M_acc = 50;
  float *acc_bin_data = new float[M_acc];
  for (int i=0;i<M_acc;i++)
  {
    acc_bin_data[i] = UAV::acc_gen_val(i);
  }

  std::cout<<"Allocating device memory"<<std::endl;
  //------

  void * vel_bins = factory.make_cuda_matrix<1,float>(&M_vel,vel_bin_data);
  void * acc_bins = factory.make_cuda_matrix<1,float>(&M_acc,acc_bin_data);

  //------
  size_t M_S[2] = {M_vel,M_acc};

  void * S_1 = factory.make_cuda_matrix<2,float>(M_S);
  void * S_2 = factory.make_cuda_matrix<2,float>(M_S);
  void * S_A = factory.make_cuda_matrix<2,UAV::UAVModel::Input>(M_S);

  //------
  std::cout<<"Start dynamic programming"<<std::endl;
  VoidPtrCarrier ptr_car;
  ptr_car[0] = S_A;
  ptr_car[1] = S_1;
  ptr_car[2] = S_2;
  ptr_car[3] = vel_bins;
  ptr_car[4] = acc_bins;

  auto start = std::chrono::steady_clock::now();
  GPU_DP::program_vel<UAV::UAVModel::Input>(ptr_car, M_S);
  auto end = std::chrono::steady_clock::now();
  std::cout << "Consumed: "
            << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
            << " s" << std::endl;

  //------
  std::cout<<"Write data to files"<<std::endl;

//  factory.write_cuda_matrix<2,UAV::UAVModel::Input>(S_A,"/home/sp/cpc_ws/SA.dat");
 // factory.write_cuda_matrix<1,float>(vel_bins,"/home/sp/cpc_ws/vel_bin.dat");
  //factory.write_cuda_matrix<1,float>(acc_bins,"/home/sp/cpc_ws/acc_bin.dat");
    factory.write_cuda_matrix<2,UAV::UAVModel::Input>(S_A,"/home/uas/yzchen_ws/SA.dat");
  factory.write_cuda_matrix<1,float>(vel_bins,"/home/uas/yzchen_ws/vel_bin.dat");
  factory.write_cuda_matrix<1,float>(acc_bins,"/home/uas/yzchen_ws/acc_bin.dat");

  std::cout<<"Release the resources"<<std::endl;
  factory.free_cuda_matrix<1,float>(vel_bins);
  factory.free_cuda_matrix<1,float>(acc_bins);
  factory.free_cuda_matrix<2,float>(S_1);
  factory.free_cuda_matrix<2,float>(S_2);
  factory.free_cuda_matrix<2,UAV::UAVModel::Input>(S_A);

  delete [] vel_bin_data;
  delete [] acc_bin_data;

}

int main(int argc, char **argv)
{
  std::cout<<"Start the dynamic programming"<<std::endl;
  position_program();
  std::cout<<"Finish the dynamic programming"<<std::endl;
  return 0;
}
