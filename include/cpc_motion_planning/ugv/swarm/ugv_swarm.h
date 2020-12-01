#ifndef UGV_SWARM_H
#define UGV_SWARM_H

#include <curand_kernel.h>
#include <cuda_math/cuda_matrix.cuh>
#include <cpc_motion_planning/ugv/model/ugv_model.h>
namespace UGV
{
template <int STEP>
class UGVSwarm
{
public:
  struct Trace
  {
    float3 site[STEP];

    __device__ __host__
    Trace()
    {
      for (unsigned int i=0;i<STEP;i++)
      {
        site[i] = make_float3(0,0,0);
      }
    }

    __device__ __host__
    float3& operator[](unsigned int i)
    {
      return site[i];
    }

    __device__ __host__
    const float3& operator[](unsigned int i) const
    {
      return site[i];
    }

    __device__ __host__
    Trace operator-(Trace const &rhs) const
    {
      Trace tmp;
      for (unsigned int i=0;i<STEP;i++)
      {
        tmp[i] = site[i] - rhs[i];
      }
      return tmp;
    }
    //---
    __device__ __host__
    Trace operator+(Trace const &rhs) const
    {
      Trace tmp;
      for (unsigned int i=0;i<STEP;i++)
      {
        tmp[i] = site[i] + rhs[i];
      }
      return tmp;
    }
    //---
    __device__ __host__
    Trace operator*(float const &rhs) const
    {
      Trace tmp;
      for (unsigned int i=0;i<STEP;i++)
      {
        tmp[i] = site[i] * rhs;
      }
      return tmp;
    }
    //---
    __device__ __host__
    Trace operator/(float const &rhs) const
    {
      Trace tmp;
      for (unsigned int i=0;i<STEP;i++)
      {
        tmp[i] = site[i] / rhs;
      }
      return tmp;
    }
    //---
    __device__ __host__
    Trace& operator=(Trace const &rhs)
    {
      for (unsigned int i=0;i<STEP;i++)
      {
        site[i] = rhs[i];
      }
      return *this;
    }

    //---
    __device__ __host__
    float square()
    {
      float square = 0;
      for (unsigned int i=0;i<STEP;i++)
      {
        square += dot(site[i],site[i]);
      }
      return square;
    }
  };

  struct Particle
  {
    Trace curr_loc;  //location
    Trace best_loc;  //self best
    Trace ptcl_vel;  //particle velocity
    float best_cost; //self best cost
    bool collision;  //marks the particle leads to an inevitable collision state
    curandState rs;  //random state
  };

  UGVSwarm()
  {
    steps = STEP;
    step_dt = PSO::PSO_TOTAL_T/static_cast<float>(steps);
  }

  ~UGVSwarm()
  {

  }

  //---
  __host__ __device__
  void bound_ptcl_velocity(Particle &p)
  {
    for (int n = 0; n < STEP; n++)
    {
      PSO::bound_between(p.ptcl_vel.site[n].x, -0.3f, 0.3f);
      PSO::bound_between(p.ptcl_vel.site[n].y, -0.3f, 0.3f);
    }
  }

  //---
  __host__ __device__
  void bound_ptcl_location(const UGVModel::State &s_ini, Particle &p)
  {
    for (int n = 0; n < STEP; n++)
    {
      PSO::bound_between(p.curr_loc.site[n].x,  -2.0f,  2.0f);
      PSO::bound_between(p.curr_loc.site[n].y,  -1.0f,  1.0f);    }
  }

  //---
  __host__ __device__
  void initialize_a_particle(const UGVModel::State &s_ini, Particle &p)
  {
    for (int i=0; i< STEP; i++)
    {
      p.curr_loc.site[i].x = PSO::rand_float_gen(&(p.rs), -2.0f,  2.0f); // station target
      p.curr_loc.site[i].y = PSO::rand_float_gen(&(p.rs), -1.0f,  1.0f); // theta target
      p.curr_loc.site[i].z = 0;

      p.ptcl_vel.site[i].x = PSO::rand_float_gen(&(p.rs), -1.0f, 1.0f);
      p.ptcl_vel.site[i].y = PSO::rand_float_gen(&(p.rs), -1.0f, 1.0f);
      p.ptcl_vel.site[i].z = 0;
    }
    bound_ptcl_velocity(p);
    bound_ptcl_location(s_ini,p);

    p.best_loc = p.curr_loc;
    p.collision = false;
    //printf("%f\n",p.best_loc[0].x);
  }

  Particle *ptcls;
  int ptcl_size;
  int steps;
  float step_dt;
};
}
#endif // UGV_SWARM_H
