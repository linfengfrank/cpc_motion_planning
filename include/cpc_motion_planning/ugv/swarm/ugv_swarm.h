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
    step_dt = 0.5f;
    total_t = steps * step_dt;
    var = make_float3(1,1,1);
  }

  ~UGVSwarm()
  {

  }

  void set_step_dt(int steps_, float dt)
  {
    steps = min(steps_,STEP);
    step_dt = dt;
    total_t = steps * step_dt;
  }

  void set_var(float3 var_)
  {
    var = var_;
  }

  //---
  __host__ __device__
  void bound_ptcl_velocity(Particle &p)
  {
    for (int n = 0; n < STEP; n++)
    {
      PSO::bound_between(p.ptcl_vel.site[n].x, -0.3f, 0.3f);
      PSO::bound_between(p.ptcl_vel.site[n].y, -0.3f, 0.3f);
      PSO::bound_between(p.ptcl_vel.site[n].z, -0.3f, 0.3f);
    }
  }

  //---
  __host__ __device__
  void bound_ptcl_location(const UGVModel::State &s_ini, Particle &p)
  {
    for (int n = 0; n < STEP; n++)
    {
      PSO::bound_between(p.curr_loc.site[n].x,  -var.x,  var.x);
      PSO::bound_between(p.curr_loc.site[n].y,  -var.y,  var.y);
      PSO::bound_between(p.curr_loc.site[n].z,  -1,  1);
    }
  }

  //---
  __host__ __device__
  void initialize_a_particle(const UGVModel::State &s_ini, Particle &p)
  {
    for (int i=0; i< STEP; i++)
    {
      p.curr_loc.site[i].x = PSO::rand_float_gen(&(p.rs), -var.x,  var.x); // station target
      p.curr_loc.site[i].y = PSO::rand_float_gen(&(p.rs), -var.y,  var.y); // theta target
      p.curr_loc.site[i].z = PSO::rand_float_gen(&(p.rs), -1.0f, 1.0f);;

      p.ptcl_vel.site[i].x = PSO::rand_float_gen(&(p.rs), -0.3f, 0.3f);
      p.ptcl_vel.site[i].y = PSO::rand_float_gen(&(p.rs), -0.3f, 0.3f);
      p.ptcl_vel.site[i].z = PSO::rand_float_gen(&(p.rs), -0.3f, 0.3f);;
    }
    bound_ptcl_velocity(p);
    bound_ptcl_location(s_ini,p);

    p.best_loc = p.curr_loc;
    p.collision = false;
    //printf("%f\n",p.best_loc[0].x);
  }

  Particle *ptcls;
  Particle *best_ptcl;
  int ptcl_size;
  int steps;
  float step_dt;
  float total_t;
  float3 var;
};
}
#endif // UGV_SWARM_H
