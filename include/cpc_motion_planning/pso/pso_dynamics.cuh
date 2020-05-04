#ifndef PSO_DYNAMICS_CUH
#define PSO_DYNAMICS_CUH

#include <cpc_motion_planning/pso/pso_utilities.cuh>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/helper_math.h>
#include <cuda_geometry/cuda_edtmap.cuh>
namespace PSO
{
struct State
{
    float3 p;
    float3 v;
    float3 a;
    __host__ __device__
    State():
      p(make_float3(0,0,0)),
      v(make_float3(0,0,0)),
      a(make_float3(0,0,0))
    {
    }
};

struct Target
{
  State s;
  bool oa;
};


__host__ __device__ __forceinline__
void model_forward(State &s, const float3 &u, const float &dt)
{
  // x
  s.p.x = s.p.x + s.v.x*dt + 0.5f*s.a.x*dt*dt + 1.0f/6.0f*u.x*dt*dt*dt;
  s.v.x = s.v.x + s.a.x*dt + 0.5f*u.x*dt*dt;
  s.a.x = s.a.x + u.x*dt;

  // y
  s.p.y = s.p.y + s.v.y*dt + 0.5f*s.a.y*dt*dt + 1.0f/6.0f*u.y*dt*dt*dt;
  s.v.y = s.v.y + s.a.y*dt + 0.5f*u.y*dt*dt;
  s.a.y = s.a.y + u.y*dt;

  // z
  s.p.z = s.p.z + s.v.z*dt + 0.5f*s.a.z*dt*dt + 1.0f/6.0f*u.z*dt*dt*dt;
  s.v.z = s.v.z + s.a.z*dt + 0.5f*u.z*dt*dt;
  s.a.z = s.a.z + u.z*dt;


}

__host__ __device__ __forceinline__
float3 dp_control(const State &s, const float3 &site, VoidPtrCarrier &aux_data, const UniformBinCarrier &ubc)
{
  // Cast all the needed pointers
  CUDA_MAT::Mat3Act *S_A = static_cast<CUDA_MAT::Mat3Act*>(aux_data[0]);

  dp_action u[3];
  float s_relative[3];

  // X axis
  s_relative[0] = s.p.x - site.x; // relative position
  s_relative[1] = s.v.x; // relative velocity
  s_relative[2] = s.a.x; // relative velocity
  u[0] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A, ubc);

  //printf("%f \n",s_relative[0]);

  // Y axis
  s_relative[0] = s.p.y - site.y; // relative position
  s_relative[1] = s.v.y; // relative velocity
  s_relative[2] = s.a.y; // relative velocity
  u[1] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A, ubc);

  // Z axis
  s_relative[0] = s.p.z - site.z; // relative position
  s_relative[1] = s.v.z; // relative velocity
  s_relative[2] = s.a.z; // relative velocity
  u[2] = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A, ubc);

  return make_float3(u[0].jerk, u[1].jerk, u[2].jerk);
}

__host__ __device__ __forceinline__
float process_cost(const State &s, const Target &goal, const EDTMap &map)
{
  float cost = 0;
  float3 dist_err = s.p - goal.s.p;
  cost += 0.5f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y + 3*dist_err.z*dist_err.z);
  cost += 0.1f*sqrt(s.v.x*s.v.x + s.v.y*s.v.y + s.v.z*s.v.z);
  cost += 0.1f*sqrt(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

  if (goal.oa)
  {
    int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
    int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
    int iz = floor( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5);

    if (ix<0 || ix>=map.m_map_size.x ||
        iy<0 || iy>=map.m_map_size.y ||
        iz<0 || iz>=map.m_map_size.z)
    {
      cost += 100;
      return cost;
    }

#ifdef  __CUDA_ARCH__
    float rd = map.edt_const_at(ix,iy,iz).d*0.2;
    cost += exp(-6*rd)*400;

    if (rd < 0.6)
      cost += 100;

    if (!map.edt_const_at(ix,iy,iz).s)
        cost += 70;
#endif
  }

  return  cost;
}

__host__ __device__ __forceinline__
float final_cost(const State &s, const Target &goal, const EDTMap &map)
{
  float cost = 0;
  float3 dist_err = s.p - goal.s.p;
  cost += 1.0f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y + 3*dist_err.z*dist_err.z);
  //cost += 0.1f*sqrt(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

  if (goal.oa)
  {
    int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
    int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
    int iz = floor( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5);

    if (ix<0 || ix>=map.m_map_size.x ||
        iy<0 || iy>=map.m_map_size.y ||
        iz<0 || iz>=map.m_map_size.z)
    {
      cost += 100;
      return cost;
    }

#ifdef  __CUDA_ARCH__
    float rd = map.edt_const_at(ix,iy,iz).d*0.2;
    cost += exp(-6*rd)*400;

    if (rd < 0.6)
      cost += 100;

    if (!map.edt_const_at(ix,iy,iz).s)
        cost += 70;
#endif
  }

  return  cost;
}

//---
__host__ __device__ __forceinline__
void bound_ptcl_velocity(Particle &p)
{
  for (int n = 0; n < PSO_STEPS; n++)
  {
    bound_between(p.ptcl_vel.site[n].x, -0.5f, 0.5f);
    bound_between(p.ptcl_vel.site[n].y, -0.5f, 0.5f);
    bound_between(p.ptcl_vel.site[n].z, -0.5f, 0.5f);
  }
}

//---
__host__ __device__ __forceinline__
void bound_ptcl_location(Particle &p, const State &s)
{
  for (int n = 0; n < PSO_STEPS; n++)
  {
    bound_between(p.curr_loc.site[n].x,  s.p.x-9.0f,  s.p.x+9.0f);
    bound_between(p.curr_loc.site[n].y,  s.p.y-9.0f,  s.p.y+9.0f);
    bound_between(p.curr_loc.site[n].z,  s.p.z-9.0f,  s.p.z+9.0f);
    //bound_between(p.curr_loc.site[n].z,  1.6f,  1.65f);
  }
}

//---
__host__ __device__ __forceinline__
void initialize_a_particle(const State &s, Particle &p)
{
  for (int i=0; i< PSO_STEPS; i++)
  {
    p.curr_loc.site[i].x = rand_float_gen(&(p.rs), s.p.x-6, s.p.x+6); // station target
    p.curr_loc.site[i].y = rand_float_gen(&(p.rs), s.p.y-6, s.p.y+6); // station target
    p.curr_loc.site[i].z = rand_float_gen(&(p.rs), s.p.z-6, s.p.z+6); // station target

    p.ptcl_vel.site[i].x = rand_float_gen(&(p.rs), -1.0f, 1.0f);
    p.ptcl_vel.site[i].y = rand_float_gen(&(p.rs), -1.0f, 1.0f);
    p.ptcl_vel.site[i].z = rand_float_gen(&(p.rs), -1.0f, 1.0f);
  }
  bound_ptcl_velocity(p);
  bound_ptcl_location(p, s);

  p.best_loc = p.curr_loc;
  //printf("%f\n",p.best_loc[0].x);
}
}
#endif // PSO_DYNAMICS_CUH
