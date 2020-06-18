#include <ubfs/bfs_mid.cuh>

void ubfs_sssp3d_wrapper(int3 &src,
                         NF1Map3D * nf1map3d)
{
  ubfs::ubfs_sssp<NF1Map3D>(src,nf1map3d->num_dirs_3d,nf1map3d->dirs_3d,nf1map3d,*(nf1map3d->ugraph));

}
