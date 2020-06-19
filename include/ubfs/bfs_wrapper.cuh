#ifndef BFS_WRAPPER_CUH
#define BFS_WRAPPER_CUH


#include <ubfs/cuda_mid_map.cuh>



class ubfs_cls
{
public:
  ubfs_cls(int ne,int nv);
  void ubfs_sssp3d_wrapper(int3 &src,
                           NF1Map3D* midmap3d);
  ubfs::ubfsGraph<int3>* ugraph;

};



#endif // BFS_WRAPPER_CUH
