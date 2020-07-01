#ifndef BFS_MID_CUH
#define BFS_MID_CUH



#include <ubfs/cuda_mid_map.cuh>
#include <ubfs/uiucbfs.cuh>
#include <ubfs/timer.h>

namespace ubfs {

__global__
void set_goal_cost(NF1Map3D midmap3d, int3 crd)
{
  midmap3d.goalCost(crd);
}
template <class Mtype>
void ubfs_sssp(int3 &src, int num_dirs,const int3* dirs,
              Mtype* midmap3d,ubfsGraph<int3> &ugraph)
{
  int3 * d_q1= thrust::raw_pointer_cast(&ugraph.q1_shared[0]);
  int3 * d_q2= thrust::raw_pointer_cast(&ugraph.q2_shared[0]);

  ugraph.tail_shared[0]= 1;
  ugraph.q1_shared[0] = src;
  //whether or not to adjust "k", see comment on "BFS_kernel_multi_blk_inGPU" for more details
  int * num_td;//number of threads
  cudaMalloc((void**) &num_td, sizeof(int));


  int num_t;//number of threads
  int k=0;//BFS level index
  int num_of_blocks;
  int num_of_threads_per_block;


  // set goal cost first
  set_goal_cost<<<1,1>>>(*midmap3d,ugraph.q1_shared[0]);
  GpuTimer tm1;
  tm1.Start();
  do
  {
    num_t=ugraph.tail_shared[0];
    ugraph.tail_shared[0]=0;

    if(num_t == 0){//frontier is empty
      cudaFree(num_td);
      break;
    }

    num_of_blocks = 1;
    num_of_threads_per_block = num_t;
    if(num_of_threads_per_block <NUM_BIN)
      num_of_threads_per_block = NUM_BIN;
    if(num_t>MAX_THREADS_PER_BLOCK)
    {
      num_of_blocks = (int)ceil(num_t/(double)MAX_THREADS_PER_BLOCK);
      num_of_threads_per_block = MAX_THREADS_PER_BLOCK;
    }
    if(num_of_blocks == 1)//will call "BFS_in_GPU_kernel"
      num_of_threads_per_block = MAX_THREADS_PER_BLOCK;
    if(num_of_blocks >1 && num_of_blocks <= NUM_SM)// will call "BFS_kernel_multi_blk_inGPU"
      num_of_blocks = NUM_SM;

    //assume "num_of_blocks" can not be very large
    dim3  grid( num_of_blocks, 1, 1);
    dim3  threads( num_of_threads_per_block, 1, 1);

    if(k%2 == 0){
      if(num_of_blocks == 1){
        BFS_in_GPU_kernel<int3,Mtype><<< grid, threads >>>(ugraph,d_q1,d_q2, *midmap3d,
                                                    num_dirs,dirs,num_t , GRAY0,k);
      }
      else if(num_of_blocks <= NUM_SM){
        (cudaMemcpy(num_td,&num_t,sizeof(int),
            cudaMemcpyHostToDevice));
        BFS_kernel_multi_blk_inGPU<int3,Mtype>
        <<< grid, threads >>>(ugraph,d_q1,d_q2, *midmap3d,
                              num_dirs,dirs,num_td, GRAY0,k);

        int switch_k= ugraph.switchk_shared[0];
        if(!switch_k){
          k--;
        }
      }
      else{
        BFS_kernel<int3,Mtype><<< grid, threads >>>(ugraph,d_q1,d_q2, *midmap3d,
                                             num_dirs,dirs,num_t, GRAY0,k);
      }
    }
    else{
      if(num_of_blocks == 1){
        BFS_in_GPU_kernel<int3,Mtype><<< grid, threads >>>(ugraph,d_q2,d_q1, *midmap3d,
                                                    num_dirs,dirs,num_t, GRAY1,k);
      }
      else if(num_of_blocks <= NUM_SM){
        (cudaMemcpy(num_td,&num_t,sizeof(int),
            cudaMemcpyHostToDevice));
        BFS_kernel_multi_blk_inGPU<int3,Mtype>
        <<< grid, threads >>>(ugraph,d_q2,d_q1,*midmap3d,
                              num_dirs,dirs,num_td, GRAY1,k);

        int switch_k= ugraph.switchk_shared[0];
        if(!switch_k){
          k--;
        }
      }
      else{
        BFS_kernel<int3,Mtype><<< grid, threads >>>(ugraph,d_q2,d_q1, *midmap3d,
                                             num_dirs,dirs,num_t,  GRAY1,k);
      }
    }
    k++;

    int h_overflow= ugraph.overflow_shared[0];
    if(h_overflow) {
      printf("Error: local queue was overflow. Need to increase W_LOCAL_QUEUE\n");
      return;
    }

  } while(1);
  tm1.Stop();
  std::cout<<"--- BFS total time is "<<float(tm1.Elapsed())<<" ms"<<std::endl;
}


template <class Mtype>
void ubfs_wave(thrust::device_vector<int3> & obs_vec, int nNodes,int num_dirs,const int3* dirs,
              Mtype* midmap3d,ubfsGraph<int3> &ugraph)
{
  int3 * d_q1= thrust::raw_pointer_cast(&ugraph.q1_shared[0]);
  int3 * d_q2= thrust::raw_pointer_cast(&ugraph.q2_shared[0]);
  // copy obs to queue!
  thrust::copy(obs_vec.begin(),obs_vec.begin()+nNodes,ugraph.q1_shared.begin());
  ugraph.tail_shared[0]= nNodes;
  //whether or not to adjust "k", see comment on "BFS_kernel_multi_blk_inGPU" for more details
  int * num_td;//number of threads
  cudaMalloc((void**) &num_td, sizeof(int));


  int num_t;//number of threads
  int k=0;//BFS level index
  int num_of_blocks;
  int num_of_threads_per_block;


  GpuTimer tm1;
  tm1.Start();
  do
  {
    num_t=ugraph.tail_shared[0];
    ugraph.tail_shared[0]=0;

    if(num_t == 0){//frontier is empty
      cudaFree(num_td);
      break;
    }

    num_of_blocks = 1;
    num_of_threads_per_block = num_t;
    if(num_of_threads_per_block <NUM_BIN)
      num_of_threads_per_block = NUM_BIN;
    if(num_t>MAX_THREADS_PER_BLOCK)
    {
      num_of_blocks = (int)ceil(num_t/(double)MAX_THREADS_PER_BLOCK);
      num_of_threads_per_block = MAX_THREADS_PER_BLOCK;
    }
    if(num_of_blocks == 1)//will call "BFS_in_GPU_kernel"
      num_of_threads_per_block = MAX_THREADS_PER_BLOCK;
    if(num_of_blocks >1 && num_of_blocks <= NUM_SM)// will call "BFS_kernel_multi_blk_inGPU"
      num_of_blocks = NUM_SM;

    //assume "num_of_blocks" can not be very large
    dim3  grid( num_of_blocks, 1, 1);
    dim3  threads( num_of_threads_per_block, 1, 1);

    if(k%2 == 0){
      if(num_of_blocks == 1){
        BFS_in_GPU_kernel<int3,Mtype><<< grid, threads >>>(ugraph,d_q1,d_q2, *midmap3d,
                                                    num_dirs,dirs,num_t , GRAY0,k,true);
      }
      else if(num_of_blocks <= NUM_SM){
        (cudaMemcpy(num_td,&num_t,sizeof(int),
            cudaMemcpyHostToDevice));
        BFS_kernel_multi_blk_inGPU<int3,Mtype>
        <<< grid, threads >>>(ugraph,d_q1,d_q2, *midmap3d,
                              num_dirs,dirs,num_td, GRAY0,k,true);

        int switch_k= ugraph.switchk_shared[0];
        if(!switch_k){
          k--;
        }
      }
      else{
        BFS_kernel<int3,Mtype><<< grid, threads >>>(ugraph,d_q1,d_q2, *midmap3d,
                                             num_dirs,dirs,num_t, GRAY0,k,true);
      }
    }
    else{
      if(num_of_blocks == 1){
        BFS_in_GPU_kernel<int3,Mtype><<< grid, threads >>>(ugraph,d_q2,d_q1, *midmap3d,
                                                    num_dirs,dirs,num_t, GRAY1,k,true);
      }
      else if(num_of_blocks <= NUM_SM){
        (cudaMemcpy(num_td,&num_t,sizeof(int),
            cudaMemcpyHostToDevice));
        BFS_kernel_multi_blk_inGPU<int3,Mtype>
        <<< grid, threads >>>(ugraph,d_q2,d_q1,*midmap3d,
                              num_dirs,dirs,num_td, GRAY1,k,true);

        int switch_k= ugraph.switchk_shared[0];
        if(!switch_k){
          k--;
        }
      }
      else{
        BFS_kernel<int3,Mtype><<< grid, threads >>>(ugraph,d_q2,d_q1, *midmap3d,
                                             num_dirs,dirs,num_t,  GRAY1,k,true);
      }
    }
    k++;

    int h_overflow= ugraph.overflow_shared[0];
    if(h_overflow) {
      printf("Error: local queue was overflow. Need to increase W_LOCAL_QUEUE\n");
      return;
    }

  } while(1);
  tm1.Stop();
  std::cout<<"<<<<- OBS-BFS total time is "<<float(tm1.Elapsed())<<" ms>>>>"<<std::endl;
}

}

#endif // BFS_MID_CUH
