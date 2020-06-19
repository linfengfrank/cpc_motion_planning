#include <ubfs/bfs_mid.cuh>
#include <ubfs/bfs_wrapper.cuh>





ubfs_cls::ubfs_cls(int ne, int nv)
{
  ugraph = new ubfs::ubfsGraph<int3> (ne,nv);
}

void ubfs_cls::ubfs_sssp3d_wrapper(int3 &src,
                         NF1Map3D * nf1map3d)
{
  ubfs::ubfs_sssp<NF1Map3D>(src,nf1map3d->num_dirs_3d,nf1map3d->d_dirs_3d,nf1map3d,*(ugraph));

}







