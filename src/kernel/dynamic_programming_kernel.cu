#include <cpc_motion_planning/dynamic_programming.h>
#define DT 0.05
namespace GPU_DP
{

__global__
void test(CUDA_MAT::Mat4Act S_A, CUDA_MAT::Mat4f S_old, CUDA_MAT::Mat4f S_new, CUDA_MAT::Vecf bin_p, CUDA_MAT::Vecf bin_v, CUDA_MAT::Vecf bin_theta, CUDA_MAT::Vecf bin_w)
{
  float s_curr[4];
  s_curr[0] =  -2.5f + 0.1f*blockIdx.x;
  s_curr[1] =  -2.5f + 0.1f*blockIdx.y;
  s_curr[2] =  -2.5f + 0.1f*blockIdx.z;
  s_curr[3] =  -2.5f + 0.1f*threadIdx.x;


  float val;
  float s_next[4];
  float val_min = 10000;
  dp_action best_action;
  bool updated = false;

  for (float acc=-2;acc<2;acc+=0.2)
  {
    for (float alpha=-5;alpha<5;alpha+=0.3)
    {
      s_next[0] = s_curr[0] + s_curr[1]*DT + 0.5*acc*DT*DT;
      s_next[1] = s_curr[1] + acc*DT;
      s_next[2] = s_curr[2] + s_curr[3]*DT + 0.5*alpha*DT*DT;
      s_next[3] = s_curr[3] + alpha*DT;
      val = acc*acc + alpha*alpha + CUDA_MAT::get_value(s_next,S_old,bin_p,bin_v,bin_theta,bin_w) +
          s_next[0]*s_next[0] + s_next[1]*s_next[1] +s_next[2]*s_next[2] + s_next[3]*s_next[3];
      if (val < val_min)
      {
        updated = true;
        val_min = val;
        best_action.acc = acc;
        best_action.alpha = alpha;
      }
    }
  }

  CUDA_MAT::mat4f_get_val(blockIdx.x,blockIdx.y,blockIdx.z,threadIdx.x,S_new) = val_min;

  if (updated)
    CUDA_MAT::mat4act_get_val(blockIdx.x,blockIdx.y,blockIdx.z,threadIdx.x,S_A) = best_action;

  //printf("%f\n",val);
}

void program(const CUDA_MAT::Mat4Act &S_A, const CUDA_MAT::Mat4f &S_1, const CUDA_MAT::Mat4f &S_2, const CUDA_MAT::Vecf &bin_p, const CUDA_MAT::Vecf &bin_v,
             const CUDA_MAT::Vecf &bin_theta, const CUDA_MAT::Vecf &bin_w)
{
  dim3 grid_size;
  grid_size.x = 50;
  grid_size.y = 50;
  grid_size.z = 50;

  dim3 block_size;
  block_size.x = 50;
  block_size.y = 1;
  block_size.z = 1;

  for (int i=0; i<100; i++)
  {
    printf("Iteration %d\n",i);
    if (i % 2 == 0)
      test<<<grid_size,block_size>>>(S_A,S_1,S_2,bin_p,bin_v,bin_theta,bin_w);
    else
      test<<<grid_size,block_size>>>(S_A,S_2,S_1,bin_p,bin_v,bin_theta,bin_w);

    cudaDeviceSynchronize();
  }


}
}
