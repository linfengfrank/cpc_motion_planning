#pragma once


#include "ubfs_config.h"
#include <ubfs/cuda_mid_map.cuh>
namespace ubfs
{




// A group of local queues of node IDs, used by an entire thread block.
// Multiple queues are used to reduce memory contention.
// Thread i uses queue number (i % NUM_BIN).
template <class Ktype>
struct LocalQueues {
  // tail[n] is the index of the first empty array in elems[n]
  int tail[NUM_BIN];

  // Queue elements.
  // The contents of queue n are elems[n][0 .. tail[n] - 1].
  Ktype elems[NUM_BIN][W_QUEUE_SIZE];

  // The number of threads sharing queue n.  We use this number to
  // compute a reduction over the queue.
  int sharers[NUM_BIN];

  // Initialize or reset the queue at index 'index'.
  // Normally run in parallel for all indices.
  __device__ void reset(int index, dim3 block_dim) {
    tail[index] = 0;		// Queue contains nothing

    // Number of sharers is (threads per block / number of queues)
    // If division is not exact, assign the leftover threads to the first
    // few queues.
    sharers[index] =
        (block_dim.x >> EXP) +   // block_dim/8
        (threadIdx.x < (block_dim.x & MOD_OP));
  }

  // Append 'value' to queue number 'index'.  If queue is full, the
  // append operation fails and *overflow is set to 1.
  __device__ void append(int index, int *overflow, Ktype value) {
    // Queue may be accessed concurrently, so
    // use an atomic operation to reserve a queue index.
    int tail_index = atomicAdd(&tail[index], 1);
    if (tail_index >= W_QUEUE_SIZE)
      *overflow = 1;
    else
      elems[index][tail_index] = value;
  }

  // Perform a scan on the number of elements in queues in a a LocalQueue.
  // This function should be executed by one thread in a thread block.
  //
  // The result of the scan is used to concatenate all queues; see
  // 'concatenate'.
  //
  // The array prefix_q will hold the scan result on output:
  // [0, tail[0], tail[0] + tail[1], ...]
  //
  // The total number of elements is returned.
  __device__ int size_prefix_sum(int (&prefix_q)[NUM_BIN]) {
    prefix_q[0] = 0;
    for(int i = 1; i < NUM_BIN; i++){
      prefix_q[i] = prefix_q[i-1] + tail[i-1];
    }
    return prefix_q[NUM_BIN-1] + tail[NUM_BIN-1];
  }

  // Concatenate and copy all queues to the destination.
  // This function should be executed by all threads in a thread block.
  //
  // prefix_q should contain the result of 'size_prefix_sum'.
  __device__ void concatenate(Ktype *dst, int (&prefix_q)[NUM_BIN]) {
    // Thread n processes elems[n % NUM_BIN][n / NUM_BIN, ...]
    int q_i = threadIdx.x & MOD_OP; // w-queue index, idx of row
    int local_shift = threadIdx.x >> EXP; // shift within a w-queue, idx of col

    while(local_shift < tail[q_i]){
      dst[prefix_q[q_i] + local_shift] = elems[q_i][local_shift];

      //multiple threads are copying elements at the same time,
      //so we shift by multiple elements for next iteration
      local_shift += sharers[q_i]; // 8*64>512, so it is out of bound???
    }
  }
};

/// TODO: 2 layer BFS
__device__
float fatomicMin(float *addr,float value){
  float old = *addr, assumed;
  if(old<=value) return old;
  do {
    assumed = old;
    // addr == assumed? min(value, assumed)): addr     ret: addr
    old = atomicCAS((int*)addr, __float_as_int(assumed), __float_as_int(min(value, assumed)));
  }while(old!=assumed);// changed by other thread: next turn, old==assumed ;changed by this thread: next turn;
  return old;
}
__device__ void
visit_free(int3 cur_buf,
           int index,
           LocalQueues<int3> &local_q,
           int *overflow,
           NF1Map3D &local_map,
           int num_dirs, const int3* dirs,
           int gray_shade, float gridstep=0.2)
{

  local_map.level(cur_buf.x,cur_buf.y,cur_buf.z)=BLACK;// Mark this node as visited
  float cur_cost = local_map.cost(cur_buf.x,cur_buf.y,cur_buf.z);
  // traverse the neighbours
  for(int i=0;i<num_dirs;i++)
  {
    int3 nbr_buf=cur_buf+dirs[i];
    if(!local_map.isCoordInsideLocalMap(nbr_buf))
      continue;

//    if(local_map.level(nbr_buf.x,nbr_buf.y,nbr_buf.z)==BLACK) // visited
//      continue;


    // update nbr cost
    float cost =cur_cost+gridstep ;
    float origin_cost = fatomicMin(&local_map.cost(nbr_buf.x,nbr_buf.y,nbr_buf.z),cost);


    if(origin_cost>cost)
    {
      int old_color = atomicExch(&local_map.level(nbr_buf.x,nbr_buf.y,nbr_buf.z),gray_shade);
      if(old_color != gray_shade) {   // free, not in this level
//        if(local_map.checkOccupancy(nbr_buf.x,nbr_buf.y,nbr_buf.z)==true)
//        {
//          //      local_map.isObs(nbr_buf.x,nbr_buf.y,nbr_buf.z) =true;
//          local_map.set_obs(nbr_buf,true);
//        }
//        else
//        {
//          //push to the queue
//          local_q.append(index, overflow, nbr_buf);  // push id to queue[index]
//        }
        //    // from obs to free, no bfs
            if(local_map.checkOccupancy(nbr_buf.x,nbr_buf.y,nbr_buf.z)==false&&
               local_map.checkOccupancy(cur_buf.x,cur_buf.y,cur_buf.z)==true)
              continue;
            local_q.append(index, overflow, nbr_buf);  // push id to queue[index]
        }
      }

    }
  }

  __device__ void
      visit_obs(int3 cur_buf,
                int index,
                LocalQueues<int3> &local_q,
                int *overflow,
                NF1Map3D &local_map,
                int num_dirs, const int3* dirs,
                int gray_shade, float gridstep=0.2)
  {
//          if(local_map.level(cur_buf.x,cur_buf.y,cur_buf.z)==BLACK) // visited
//            return;

    local_map.level(cur_buf.x,cur_buf.y,cur_buf.z)=BLACK;// Mark this node as visited
    float cur_cost = local_map.cost(cur_buf.x,cur_buf.y,cur_buf.z);
    // traverse the neighbours
    for(int i=0;i<num_dirs;i++)
    {
      int3 nbr_buf=cur_buf+dirs[i];
      if(!local_map.isCoordInsideLocalMap(nbr_buf))
        continue;

      // whether to commant it out??
      if(local_map.level(nbr_buf.x,nbr_buf.y,nbr_buf.z)==BLACK) // visited
        continue;

      // only bfs obs
      if(local_map.checkOccupancy(nbr_buf.x,nbr_buf.y,nbr_buf.z)==false)
        continue;


      // set obs flag to default
//      local_map.set_obs(nbr_buf,false);
      // update nbr cost
      float cost =cur_cost+gridstep ;
      float origin_cost = fatomicMin(&local_map.cost(nbr_buf.x,nbr_buf.y,nbr_buf.z),cost);

      if(origin_cost>cost)
      {
        int old_color = atomicExch(&local_map.level(nbr_buf.x,nbr_buf.y,nbr_buf.z),gray_shade);
        if(old_color != gray_shade) {   // not in this level
          //push to the queue
          local_q.append(index, overflow, nbr_buf);  // push id to queue[index]
        }
      }
    }


    //  }
  }
  //-------------------------------------------------
  //This is the version for one-block situation. The propagation idea is basically the same as
  //BFS_kernel.
  //The major differences are:
  // 1) This kernel can propagate though multiple BFS levels (while loop) using __synchThreads() between levels
  // 2) the intermediate queues are stored in shared memory (next_wf)
  //\param q1: the current frontier queue when the kernel is launched
  //\param q2: the new frontier queue when the  kernel returns
  //--------------------------------------------------
  template <class Ktype, class Mtype>
  __global__ void
      BFS_in_GPU_kernel(ubfsGraphBase<Ktype> ugraph,
                        Ktype *q1,
                        Ktype *q2,
                        Mtype local_map,
                        int num_dirs, const int3* dirs,
                        int no_of_nodes,
                        int gray_shade,
                        int k,
                        bool obs_mode =false)
  {
    __shared__ LocalQueues<Ktype> local_q;
    __shared__ int prefix_q[NUM_BIN];// store number of elems of each rows

    //next/new wave front
    __shared__ Ktype next_wf[MAX_THREADS_PER_BLOCK];
    __shared__ int  tot_sum;
    if(threadIdx.x == 0)
      tot_sum = 0;//total number of new frontier nodes
    while(1){//propage through multiple BFS levels until the wavfront overgrows one-block limit
      if(threadIdx.x < NUM_BIN){
        local_q.reset(threadIdx.x, blockDim);
      }
      __syncthreads();
      int tid = blockIdx.x*MAX_THREADS_PER_BLOCK + threadIdx.x;
      if( tid<no_of_nodes)
      {
        Ktype pid;  // id of nodes in frontier
        if(tot_sum == 0)//this is the first BFS level of current kernel call
          pid = q1[tid];
        else
          pid = next_wf[tid];//read the current frontier info from last level's propagation

        if(obs_mode==false)
          visit_free(pid,threadIdx.x & MOD_OP,local_q,ugraph.overflow,local_map,num_dirs,dirs,gray_shade);
        else
          visit_obs(pid,threadIdx.x & MOD_OP,local_q,ugraph.overflow,local_map,num_dirs,dirs,gray_shade);
      }
      __syncthreads();
      if(threadIdx.x == 0){
        *(ugraph.tail) = tot_sum = local_q.size_prefix_sum(prefix_q);
      }
      __syncthreads();

      if(tot_sum == 0)//the new frontier becomes empty; BFS is over
        return;
      if(tot_sum <= MAX_THREADS_PER_BLOCK){
        //the new frontier is still within one-block limit;
        //stay in current kernel
        local_q.concatenate(next_wf, prefix_q);
        __syncthreads();
        no_of_nodes = tot_sum;
        if(threadIdx.x == 0){
          if(gray_shade == GRAY0)
            gray_shade = GRAY1;
          else
            gray_shade = GRAY0;
        }
      }
      else{
        //the new frontier outgrows one-block limit; terminate current kernel
        local_q.concatenate(q2, prefix_q);
        return;
      }
    }//while

  }

  volatile __device__ int count = 0;
  volatile __device__ int no_of_nodes_vol = 0;
  volatile __device__ int stay_vol = 0;

  //Inter-block sychronization
  //This only works when there is only one block per SM
  __device__ void start_global_barrier(int fold){
    __syncthreads();

    if(threadIdx.x == 0){
      atomicAdd((int*)&count, 1);
      while( count < NUM_SM*fold){
        ;
      }
    }
    __syncthreads();

  }

  __device__ void rst_barrier(int *global_kt)
  {
    count = 0;
    *global_kt=0;
  }
  /*****************************************************************************
  This BFS kernel propagates through multiple levels using global synchronization
  The basic propagation idea is the same as "BFS_kernel"
  The major differences are:
  1) propagate through multiple levels by using GPU global sync ("start_global_barrier")
  2) use q1 and q2 alternately for the intermediate queues
  \param q1: the current frontier when the kernel is called
  \param q2: possibly the new frontier when the kernel returns depending on how many levels of propagation
            has been done in current kernel; the new frontier could also be stored in q1
  \param switch_k: whether or not to adjust the "k" value on the host side
                Normally on the host side, when "k" is even, q1 is the current frontier; when "k" is
                odd, q2 is the current frontier; since this kernel can propagate through multiple levels,
                the k value may need to be adjusted when this kernel returns.
  \param global_kt: the total number of global synchronizations,
                    or the number of times to call "start_global_barrier"
 *****************************************************************************/
  template <class Ktype,class Mtype>
  __global__ void
      BFS_kernel_multi_blk_inGPU(ubfsGraphBase<Ktype> ugraph,
                                 Ktype *q1,
                                 Ktype *q2,
                                 Mtype local_map,
                                 int num_dirs, const int3* dirs,
                                 int *no_of_nodes,   //the  frontier num
                                 int gray_shade,
                                 int k,
                                 bool obs_mode =false)
  {
    __shared__ LocalQueues<Ktype> local_q;
    __shared__ int prefix_q[NUM_BIN];
    __shared__ int shift;
    __shared__ int no_of_nodes_sm;
    __shared__ int odd_time;// the odd level of propagation within current kernel
    if(threadIdx.x == 0){
      odd_time = 1;//true;
      if(blockIdx.x == 0)
        no_of_nodes_vol = *no_of_nodes;
    }
    int kt = atomicOr(ugraph.global_kt,0);// the total count of GPU global synchronization
    while (1){//propagate through multiple levels
      if(threadIdx.x < NUM_BIN){
        local_q.reset(threadIdx.x, blockDim);
      }
      if(threadIdx.x == 0)
        no_of_nodes_sm = no_of_nodes_vol;
      __syncthreads();

      int tid = blockIdx.x*MAX_THREADS_PER_BLOCK + threadIdx.x;
      if( tid<no_of_nodes_sm)
      {
        // Read a node ID from the current input queue
        Ktype *input_queue = odd_time ? q1 : q2;
        Ktype pid= input_queue[tid];
        //			int pid = atomicOr((int *)&input_queue[tid], 0);  //????????????????

        if(obs_mode==false)
          visit_free(pid,threadIdx.x & MOD_OP,local_q,ugraph.overflow,
                     local_map,num_dirs,dirs,gray_shade);
        else
          visit_obs(pid,threadIdx.x & MOD_OP,local_q,ugraph.overflow,
                    local_map,num_dirs,dirs,gray_shade);
      }
      __syncthreads();

      // Compute size of the output and allocate space in the global queue
      if(threadIdx.x == 0){
        int tot_sum = local_q.size_prefix_sum(prefix_q);
        shift = atomicAdd(ugraph.tail, tot_sum);
      }
      __syncthreads();

      // Copy to the current output queue in global memory
      Ktype *output_queue = odd_time ? q2 : q1;
      local_q.concatenate(output_queue + shift, prefix_q);

      if(threadIdx.x == 0){
        odd_time = (odd_time+1)%2;
        if(gray_shade == GRAY0)
          gray_shade = GRAY1;
        else
          gray_shade = GRAY0;
      }

      //synchronize among all the blks
      start_global_barrier(kt+1);
      if(blockIdx.x == 0 && threadIdx.x == 0){
        stay_vol = 0;
        if(*(ugraph.tail)< NUM_SM*MAX_THREADS_PER_BLOCK && *(ugraph.tail) > MAX_THREADS_PER_BLOCK){
          stay_vol = 1;
          no_of_nodes_vol = *(ugraph.tail);
          *(ugraph.tail) = 0;
        }
      }
      start_global_barrier(kt+2);
      kt+= 2;
      if(stay_vol == 0)
      {
        if(blockIdx.x == 0 && threadIdx.x == 0)
        {
          rst_barrier(ugraph.global_kt);
          //				*global_kt = kt;
          //				*(ugraph.global_kt)= kt;
          *(ugraph.switchk) = (odd_time+1)%2;
          *no_of_nodes = no_of_nodes_vol;
        }
        return;
      }
    }
  }

  /*****************************************************************************
  This is the  most general version of BFS kernel, i.e. no assumption about #block in the grid
  \param q1: the array to hold the current frontier
  \param q2: the array to hold the new frontier
  \param g_graph_nodes: the nodes in the input graph
  \param g_graph_edges: the edges i nthe input graph
  \param g_color: the colors of nodes
  \param g_cost: the costs of nodes
  \param no_of_nodes: the number of nodes in the current frontier
  \param tail: pointer to the location of the tail of the new frontier. *tail is the size of the new frontier
  \param gray_shade: the shade of the gray in current BFS propagation. See GRAY0, GRAY1 macro definitions for more details
  \param k: the level of current propagation in the BFS tree. k= 0 for the first propagation.
 ***********************************************************************/
  template <class Ktype, class Mtype>
  __global__ void
      BFS_kernel(ubfsGraphBase<Ktype> ugraph,
                 Ktype *q1,
                 Ktype *q2,
                 Mtype local_map,
                 int num_dirs, const int3* dirs,
                 int no_of_nodes,
                 int gray_shade,
                 int k,
                 bool obs_mode =false)
  {
    __shared__ LocalQueues<Ktype> local_q;
    __shared__ int prefix_q[NUM_BIN];//the number of elementss in the w-queues ahead of
    //current w-queue, a.k.a prefix sum
    __shared__ int shift;

    if(threadIdx.x < NUM_BIN){
      local_q.reset(threadIdx.x, blockDim);
    }
    __syncthreads();

    //first, propagate and add the new frontier elements into w-queues
    int tid = blockIdx.x*MAX_THREADS_PER_BLOCK + threadIdx.x;
    if( tid < no_of_nodes)
    {
      if(obs_mode==false)
        visit_free(q1[tid],threadIdx.x & MOD_OP,local_q,ugraph.overflow,
                   local_map,num_dirs,dirs,gray_shade);
      else
        visit_obs(q1[tid],threadIdx.x & MOD_OP,local_q,ugraph.overflow,
                  local_map,num_dirs,dirs,gray_shade);
    }
    __syncthreads();

    // Compute size of the output and allocate space in the global queue
    if(threadIdx.x == 0){
      //now calculate the prefix sum
      int tot_sum = local_q.size_prefix_sum(prefix_q);
      //the offset or "shift" of the block-level queue within the
      //grid-level queue is determined by atomic operation
      shift = atomicAdd(ugraph.tail,tot_sum);
    }
    __syncthreads();

    //now copy the elements from w-queues into grid-level queues.
    //Note that we have bypassed the copy to/from block-level queues for efficiency reason
    local_q.concatenate(q2 + shift, prefix_q);
  }





}
