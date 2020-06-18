#ifndef CUDA_MID_MAP_CUH
#define CUDA_MID_MAP_CUH


#include <cuda_geometry/cuda_geometry.cuh>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <cuda_geometry/cuda_edtmap.cuh>

typedef thrust::device_system_tag device_memspace;
typedef thrust::host_system_tag host_memspace;


template <typename T, typename M>
struct ptr_type {};

template <typename T>
struct ptr_type<T, host_memspace> {
  typedef T* type;
};
template <typename T>
struct ptr_type<T, device_memspace> {
  typedef thrust::device_ptr<T> type;
};
template <typename T, typename M>
struct vector_type {};

template <typename T>
struct vector_type<T, host_memspace> {
  typedef thrust::host_vector<T> type;
};
template <typename T>
struct vector_type<T, device_memspace> {
  typedef thrust::device_vector<T> type;
};
namespace ubfs
{
template <class Ktype>
struct ubfsGraphBase
{


  // param
  int numEdges,numVertices;
  int * tail;
  int * switchk;
  int *global_kt;
  int *overflow;
  bool *isend;

  // graph
  Ktype *q1D;
  Ktype *q2D;

  ubfsGraphBase(int ne,int nv):numEdges(ne),numVertices(nv)
  {
  }



};

template <class Ktype, class memspace=device_memspace>
class ubfsGraph: public ubfsGraphBase<Ktype>
{
public:
  // arr
  typedef ubfsGraphBase<Ktype> parent_type;

  typename vector_type<Ktype,memspace>::type q1_shared;
  typename vector_type<Ktype,memspace>::type q2_shared;
  typename vector_type<int,memspace>::type tail_shared;
  typename vector_type<int,memspace>::type switchk_shared;
  typename vector_type<int,memspace>::type global_kt_shared;
  typename vector_type<int,memspace>::type overflow_shared;
  typename vector_type<bool,memspace>::type isend_shared;

  ubfsGraph(int ne,int nv):parent_type(ne,nv),
    q1_shared(nv), q2_shared(nv),
    tail_shared(1), switchk_shared(1),
    global_kt_shared(1),
    overflow_shared(1), isend_shared(1)
  {
    tail_shared[0]= 1;
    global_kt_shared[0]=0;
    overflow_shared[0]= 0;
    isend_shared[0]= false;


    using thrust::raw_pointer_cast;
    this->q1D= raw_pointer_cast(&q1_shared[0]);
    this->q2D= raw_pointer_cast(&q2_shared[0]);
    this->tail = raw_pointer_cast(&tail_shared[0]);
    this->switchk = raw_pointer_cast(&switchk_shared[0]);
    this->global_kt = raw_pointer_cast(&global_kt_shared[0]);
    this->overflow = raw_pointer_cast(&overflow_shared[0]);
    this->isend = raw_pointer_cast(&isend_shared[0]);

  }

};
}
class NF1Map3D
{
public:

  //---
  NF1Map3D(int3 m_map_size_, float grid_step_);
  //---
  ~NF1Map3D()
  {

  }
  //---
  void setup_device();


  void free_device();


  __device__
  float& cost(int x, int y, int z)
  {
          return d_cost_to_go[z*m_map_size.x*m_map_size.y+y*m_map_size.x+x];
  }

  __device__
  int& level(int x, int y, int z)
  {
          return d_color[z*m_map_size.x*m_map_size.y+y*m_map_size.x+x];
  }

  __device__
  bool& isObs(int x, int y, int z)
  {
          return d_obsflg[z*m_map_size.x*m_map_size.y+y*m_map_size.x+x];
  }

  __device__
  bool checkOccupancy(int x,int y,int z)
  {
    float dist = d_val_map[z*m_map_size.x*m_map_size.y+y*m_map_size.x+x].d;
    float real_dist = dist* grid_step;
    if (real_dist < 1)
    {
      return true;
    }else
      return false;

  }
  __device__
  bool isCoordInsideLocalMap(const int3 & s) const
  {
      if (s.x<0 || s.x>=m_map_size.x ||
              s.y<0 || s.y>=m_map_size.y ||
              s.z<0 || s.z>=m_map_size.z)
      {
          return 0;
      }
      else
      {
          return 1;
      }
  }

  __device__
  void setDefaut()
  {
    CUDA_DEV_MEMSET(d_cost_to_go,0,static_cast<size_t>(m_byte_size));
    CUDA_DEV_MEMSET(d_color,0,static_cast<size_t>(m_color_size));
    CUDA_DEV_MEMSET(d_obsflg,0,static_cast<size_t>(m_flg_size));
    CUDA_DEV_MEMSET(d_val_map,0,static_cast<size_t>(m_edt_size));

  }

public:
  SeenDist *d_val_map;
  float *d_cost_to_go;
  int *d_color;
  bool *d_obsflg;
  int3* d_dirs_3d;

  int m_edt_size;
  int m_byte_size;
  int m_color_size;
  int m_flg_size;
  int3 m_map_size;
  int m_max_width;
  int m_volume;

  float grid_step;
  const static int num_dirs_3d=6;
  const int3 dirs_3d[num_dirs_3d]={int3{-1, 0, 0},int3{1, 0, 0},int3{0, -1, 0},
                                        int3{0, 1, 0},int3{0, 0, -1},int3{0, 0, 1}};
  ubfs::ubfsGraph<int3> *ugraph;
};



#endif // CUDA_MID_MAP_CUH
