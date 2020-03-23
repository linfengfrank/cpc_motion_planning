#include <cpc_motion_planning/dynamic_programming.h>

namespace GPU_DP
{

__global__
void test(CUDA_MAT::Vecf bin)
{
  int idx = CUDA_MAT::search_idx(1.5,bin);

  printf("%d\n",idx);
}

void program(const CUDA_MAT::Vecf &bin)
{
  test<<<1,1>>>(bin);
}
}
