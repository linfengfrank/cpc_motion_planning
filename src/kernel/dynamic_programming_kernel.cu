#include <cpc_motion_planning/dynamic_programming.cuh>
#define DT 0.05
namespace GPU_DP
{
__global__
void test(VoidPtrCarrier data)
{
  CUDA_MAT::Mat3Act *S_A = static_cast<CUDA_MAT::Mat3Act*>(data[0]);
  CUDA_MAT::Mat3f *S_old = static_cast<CUDA_MAT::Mat3f*>(data[1]);
  CUDA_MAT::Mat3f *S_new = static_cast<CUDA_MAT::Mat3f*>(data[2]);
  CUDA_MAT::Vecf *bin_p = static_cast<CUDA_MAT::Vecf*>(data[3]);
  CUDA_MAT::Vecf *bin_v = static_cast<CUDA_MAT::Vecf*>(data[4]);
  CUDA_MAT::Vecf *bin_a = static_cast<CUDA_MAT::Vecf*>(data[5]);

  float s_curr[3];
  s_curr[0] =  pos_gen_val(blockIdx.x);
  s_curr[1] =  vel_gen_val(blockIdx.y);
  s_curr[2] =  acc_gen_val(threadIdx.x);


  float val;
  float s_next[3];
  float val_min = 1e6;
  float jerk;
  action best_action;
  bool updated = false;

  for (float jerk=-5;jerk<5.1;jerk+=0.2)
  {
      s_next[0] = s_curr[0] + s_curr[1]*DT + 0.5*s_curr[2]*DT*DT + 1.0f/6.0f*jerk*DT*DT*DT;
      s_next[1] = s_curr[1] + s_curr[2]*DT + 0.5*jerk*DT*DT;
      s_next[2] = s_curr[2] + jerk*DT;

      val = CUDA_MAT::get_value_3(s_next,*S_old, *bin_p, *bin_v, *bin_a);
      val += 2*jerk*jerk;
      val += 3*s_curr[0]*s_curr[0] + 0.5*s_curr[1]*s_curr[1] + 0.1*s_curr[2]*s_curr[2];
      if (s_curr[1] - 4.0 > 0)
        val += (20+80*(s_curr[1] - 4.0));

      if (s_curr[1] < -4.0)
        val += (20+80*(-s_curr[1] - 4.0));

      if (s_curr[2] - 2.5 > 0)
        val += (40+160*(s_curr[2] - 2.5));

      if (s_curr[2] < -2.5)
        val += (40+160*(-s_curr[2] - 2.5));

//      val+= 0.5*acc_tot*acc_tot;

      if (fabs(s_curr[0]) > 0.25 || fabs(s_curr[1]) > 0.25 || fabs(s_curr[2]) > 0.25)
      {
        val += 4;
      }

      if (val < val_min)
      {
        updated = true;
        val_min = val;
        best_action.jerk = jerk;
      }
  }

  CUDA_MAT::mat3f_get_val(blockIdx.x,blockIdx.y,threadIdx.x,*S_new) = val_min;

  if (updated)
    CUDA_MAT::mat3act_get_val(blockIdx.x,blockIdx.y,threadIdx.x,*S_A) = best_action;

  //printf("%f\n",val);
}

void program(VoidPtrCarrier ptr_car, size_t *bin_size)
{
  dim3 grid_size;
  grid_size.x = bin_size[0];
  grid_size.y = bin_size[1];
  grid_size.z = 1;

  dim3 block_size;
  block_size.x = bin_size[2];
  block_size.y = 1;
  block_size.z = 1;



  for (int i=0; i<100; i++)
  {
    printf("Iteration %d\n",i);
    if (i % 2 == 0)
    {
      void* tmp = ptr_car[2];
      ptr_car[2] = ptr_car[1];
      ptr_car[1] = tmp;
      test<<<grid_size,block_size>>>(ptr_car);
    }
    else
    {
      void* tmp = ptr_car[2];
      ptr_car[2] = ptr_car[1];
      ptr_car[1] = tmp;
      test<<<grid_size,block_size>>>(ptr_car);
    }

    cudaDeviceSynchronize();
  }
}
}
//template void GPU_DP::program<7>(VoidPtrCarrier<7> ptr_car, size_t *bin_size);

