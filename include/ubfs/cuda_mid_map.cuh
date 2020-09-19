#ifndef CUDA_MID_MAP_CUH
#define CUDA_MID_MAP_CUH


#include <cuda_geometry/cuda_geometry.cuh>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <ubfs/ubfs_config.h>
#include <cfloat>

#define EMPTY_KEY  int3{999999,999999,999999}
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


struct EqualTo
{
  __device__ __host__
  bool operator()(int3 a, int3 b) {
    return (a.x == b.x) &&
           (a.y == b.y) &&
           (a.z == b.z);
  }
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
    switchk_shared[0] =0;


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


  void setDefaut();

  __device__
  float& cost(int x, int y, int z)
  {
    return d_cost_to_go[z*m_map_size.x*m_map_size.y+y*m_map_size.x+x];
  }

  __device__
  int& int_cost(int x, int y, int z)
  {
    return d_cost_int[z*m_map_size.x*m_map_size.y+y*m_map_size.x+x];
  }

  __device__
  int& level(int x, int y, int z)
  {
    return d_color[z*m_map_size.x*m_map_size.y+y*m_map_size.x+x];
  }

//  __device__
//  bool& isObs(int x, int y, int z)
//  {
//    return d_obsflg[z*m_map_size.x*m_map_size.y+y*m_map_size.x+x];
//  }
  __device__
  void set_obs(int3 obs,bool isobs)
  {
    if(isobs)
      d_obs[obs.z*m_map_size.x*m_map_size.y+obs.y*m_map_size.x+obs.x]=obs;
    else
      d_obs[obs.z*m_map_size.x*m_map_size.y+obs.y*m_map_size.x+obs.x]=EMPTY_KEY;
  }
//  __device__
//  bool get_obs(int3 crd) // if obs, return true
//  {
//     return (crd.x != EMPTY_KEY.x) ||
//         (crd.y != EMPTY_KEY.y) ||
//         (crd.z != EMPTY_KEY.z);
//  }
  __device__
  float goalCost(int3 goal,float obstacle_dist=1)
  {
    int idx_1d =goal.z*m_map_size.x*m_map_size.y+goal.y*m_map_size.x+goal.x;
    float dist = d_val_map[idx_1d].d;
    dist*=grid_step;
    // Check wheter it is on any obstacle or stays in the height range
    if (dist <= obstacle_dist)
    {
      d_cost_to_go[idx_1d] = 400.0f*expf(-dist*1.5f);
//      d_obsflg[idx_1d] = true;
      d_obs[idx_1d] =goal;
    }
    else
    {
      d_cost_to_go[idx_1d] = 0.0;
//      d_obsflg[idx_1d] = false;
      d_obs[idx_1d] =EMPTY_KEY;
    }
    // for profile
    d_cost_int[idx_1d] = 0;

    return d_cost_to_go[idx_1d];
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


  __host__
  void cost_d2h()
  {
    CUDA_MEMCPY_D2H(h_cost_to_go,d_cost_to_go,static_cast<size_t>(m_byte_size));
  }

  __host__
  void obs_dense_d2h()
  {
    thrust::copy(obs_vec_dense.begin(),obs_vec_dense.end(),obs_dense_h.begin());
  }
public:
  SeenDist *d_val_map;
  float *d_cost_to_go, *h_cost_to_go;
  int *d_cost_int;
  int *d_color;
//  bool *d_obsflg;
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


  thrust::device_vector<int3> obs_vec;
  thrust::device_vector<int3> obs_vec_dense;
  thrust::host_vector<int3> obs_dense_h;
  int obs_num;
private:
  int3 *d_obs,*d_obs_dense;
};



#endif // CUDA_MID_MAP_CUH
