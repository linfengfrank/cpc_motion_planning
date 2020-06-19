#include <ubfs/cuda_mid_map.cuh>

NF1Map3D::NF1Map3D(int3 m_map_size_, float grid_step_):
  m_map_size(m_map_size_), grid_step(grid_step_)
{
  m_volume =m_map_size.x*m_map_size.y*m_map_size.z;
  m_byte_size = m_volume*static_cast<int>(sizeof(float));
  m_color_size = m_volume*static_cast<int>(sizeof(int));
  m_flg_size =m_volume*static_cast<int>(sizeof(bool));
  m_edt_size = m_volume*static_cast<int>(sizeof(SeenDist));
  m_max_width = m_map_size.x+m_map_size.y+m_map_size.z;


}

void NF1Map3D::setup_device()
{
  CUDA_ALLOC_DEV_MEM(&d_cost_to_go,static_cast<size_t>(m_byte_size));
  CUDA_DEV_MEMSET(d_cost_to_go,FLT_MAX,static_cast<size_t>(m_byte_size));

  CUDA_ALLOC_DEV_MEM(&d_color,static_cast<size_t>(m_color_size));
  CUDA_DEV_MEMSET(d_color,WHITE,static_cast<size_t>(m_color_size));

  CUDA_ALLOC_DEV_MEM(&d_obsflg,static_cast<size_t>(m_flg_size));
  CUDA_DEV_MEMSET(d_obsflg,0,static_cast<size_t>(m_flg_size));

  CUDA_ALLOC_DEV_MEM(&d_val_map,static_cast<size_t>(m_edt_size));
  CUDA_DEV_MEMSET(d_val_map,0,static_cast<size_t>(m_edt_size));

  CUDA_ALLOC_DEV_MEM(&d_dirs_3d,num_dirs_3d*sizeof(int3));
  CUDA_MEMCPY_H2D(d_dirs_3d,dirs_3d,num_dirs_3d*sizeof(int3));

  //  cudaHostAlloc((**void)&h_cost_to_go,m_byte_size,cudaHostAllocDefault);
    h_cost_to_go = new float[m_byte_size];
}

void NF1Map3D::free_device()
{
  CUDA_FREE_DEV_MEM(d_cost_to_go);
  CUDA_FREE_DEV_MEM(d_color);
  CUDA_FREE_DEV_MEM(d_obsflg);
  CUDA_FREE_DEV_MEM(d_val_map);

  CUDA_FREE_DEV_MEM(d_dirs_3d);

  delete [] h_cost_to_go;

}



