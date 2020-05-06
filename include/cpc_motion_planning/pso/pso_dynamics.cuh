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
