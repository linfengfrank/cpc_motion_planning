#include <ubfs/bfs_mid.cuh>
#include <ubfs/bfs_wrapper.cuh>


struct is_obs {
  __device__
  bool operator() (const int3 &k) {
    return (k.x!=EMPTY_KEY.x || k.y!=EMPTY_KEY.y || k.z != EMPTY_KEY.z);
  }
};


ubfs_cls::ubfs_cls(int ne, int nv)
{
  ugraph = new ubfs::ubfsGraph<int3> (ne,nv);
}

void ubfs_cls::ubfs_sssp3d_wrapper(int3 &src,
                                   NF1Map3D * nf1map3d)
{



  GpuTimer tm1;
  tm1.Start();
  nf1map3d->setDefaut();
  ubfs::ubfs_sssp<NF1Map3D>(src,nf1map3d->num_dirs_3d,nf1map3d->d_dirs_3d,nf1map3d,*(ugraph));
  tm1.Stop();
  std::cout<<"--- BFS total time is "<<float(tm1.Elapsed())<<" ms"<<std::endl;
//  cudaDeviceSynchronize();
//  auto last_it = thrust::copy_if(nf1map3d->obs_vec.begin(),nf1map3d->obs_vec.end(),
//                                 nf1map3d->obs_vec_dense.begin(),is_obs());

//  //    auto last_it_unique = thrust::unique(nf1map3d->obs_vec_dense.begin(),
//  //                                  last_it,EqualTo());

//  nf1map3d->obs_num =thrust::distance(nf1map3d->obs_vec_dense.begin(),last_it);
//  //    printf("obs length is %d\n",nf1map3d->obs_num);

//  ubfs::ubfs_wave<NF1Map3D>(nf1map3d->obs_vec_dense,  nf1map3d->obs_num,nf1map3d->num_dirs_3d,
//                            nf1map3d->d_dirs_3d,nf1map3d,*(ugraph));
//  cudaDeviceSynchronize();

}







