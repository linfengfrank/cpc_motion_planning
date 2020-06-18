#ifndef BFS_WRAPPER_CUH
#define BFS_WRAPPER_CUH


#include <ubfs/cuda_mid_map.cuh>

void ubfs_sssp3d_wrapper(int3 &src,
                         NF1Map3D* midmap3d);
#endif // BFS_WRAPPER_CUH
