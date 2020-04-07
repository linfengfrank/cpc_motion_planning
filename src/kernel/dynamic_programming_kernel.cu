#include <cpc_motion_planning/dynamic_programming.cuh>
#define DT 0.05
namespace GPU_DP
{
template <int N>
__global__
void test(VoidPtrCarrier<N> data)
{
  CUDA_MAT::Mat4Act *S_A = static_cast<CUDA_MAT::Mat4Act*>(data[0]);
  CUDA_MAT::Mat4f *S_old = static_cast<CUDA_MAT::Mat4f*>(data[1]);
  CUDA_MAT::Mat4f *S_new = static_cast<CUDA_MAT::Mat4f*>(data[2]);
  CUDA_MAT::Vecf *bin_p = static_cast<CUDA_MAT::Vecf*>(data[3]);
  CUDA_MAT::Vecf *bin_v = static_cast<CUDA_MAT::Vecf*>(data[4]);
  CUDA_MAT::Vecf *bin_theta = static_cast<CUDA_MAT::Vecf*>(data[5]);
  CUDA_MAT::Vecf *bin_w = static_cast<CUDA_MAT::Vecf*>(data[6]);

  float s_curr[4];
  s_curr[0] =  pos_gen_val(blockIdx.x);
  s_curr[1] =  vel_gen_val(blockIdx.y);
  s_curr[2] =  theta_gen_val(blockIdx.z);
  s_curr[3] =  w_gen_val(threadIdx.x);


  float val;
  float s_next[4];
  float val_min = 1e6;
  float acc_lat;
  float acc_tot;
  dp_action best_action;
  bool updated = false;

  for (float acc=-2;acc<2.1;acc+=0.2)
  {
    for (float alpha=-3;alpha<3.1;alpha+=0.3)
    {
      s_next[0] = s_curr[0] + s_curr[1]*DT + 0.5*acc*DT*DT;
      s_next[1] = s_curr[1] + acc*DT;
      s_next[2] = s_curr[2] + s_curr[3]*DT + 0.5*alpha*DT*DT;
      s_next[3] = s_curr[3] + alpha*DT;
      val = CUDA_MAT::get_value(s_next,*S_old, *bin_p, *bin_v, *bin_theta, *bin_w);
      val += 10*acc*acc + 10*alpha*alpha;
      val += 1*s_curr[0]*s_curr[0] + 0.2*s_curr[1]*s_curr[1] +s_curr[2]*s_curr[2] + 0.2*s_curr[3]*s_curr[3];
      if (s_curr[1] - 4.0 > 0)
        val += 80*(s_curr[1] - 4.0);

      if (s_curr[1] < -4.0)
        val += 80*(-s_curr[1] - 4.0);

      if (s_curr[3] - 2 > 0)
        val += 80*(s_curr[3] - 2);

      if (s_curr[3] < -2)
        val += 80*(-s_curr[3] - 2);

      acc_lat = s_curr[1]*s_curr[3];
      acc_tot = sqrt(acc_lat*acc_lat + acc*acc);

//      if (fabs(acc_lat) > 2)
//        val += 80*(fabs(acc_lat) - 2);

      if (acc_tot - 1.5 > 0)
        val += 80*(acc_tot - 1.5);

//      val+= 0.5*acc_tot*acc_tot;

      if (fabs(s_curr[0]) > 0.25 || fabs(s_curr[1]) > 0.25 || fabs(s_curr[2]) > 0.25 || fabs(s_curr[3]) > 0.25)
      {
        val += 32;
      }

      if (val < val_min)
      {
        updated = true;
        val_min = val;
        best_action.acc = acc;
        best_action.alpha = alpha;
      }
    }
  }

  CUDA_MAT::mat4f_get_val(blockIdx.x,blockIdx.y,blockIdx.z,threadIdx.x,*S_new) = val_min;

  if (updated)
    CUDA_MAT::mat4act_get_val(blockIdx.x,blockIdx.y,blockIdx.z,threadIdx.x,*S_A) = best_action;

  //printf("%f\n",val);
}

template<int N>
void program(VoidPtrCarrier<N> ptr_car, size_t *bin_size)
{
  dim3 grid_size;
  grid_size.x = bin_size[0];
  grid_size.y = bin_size[1];
  grid_size.z = bin_size[2];

  dim3 block_size;
  block_size.x = bin_size[3];
  block_size.y = 1;
  block_size.z = 1;



  for (int i=0; i<140; i++)
  {
    printf("Iteration %d\n",i);
    if (i % 2 == 0)
    {
      void* tmp = ptr_car[2];
      ptr_car[2] = ptr_car[1];
      ptr_car[1] = tmp;
      test<N><<<grid_size,block_size>>>(ptr_car);
    }
    else
    {
      void* tmp = ptr_car[2];
      ptr_car[2] = ptr_car[1];
      ptr_car[1] = tmp;
      test<N><<<grid_size,block_size>>>(ptr_car);
    }

    cudaDeviceSynchronize();
  }
}
}
template void GPU_DP::program<7>(VoidPtrCarrier<7> ptr_car, size_t *bin_size);

