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
    float2 p; //x, y
    float s; // station (longitudinal distance)
    float v; // forward speed
    float theta; // heading
    float w; // heading speed
    __host__ __device__
    State():
      p(make_float2(0,0)),
      s(0),
      v(0),
      theta(0),
      w(0)
    {
    }
};

__host__ __device__ __forceinline__
void model_forward(State &s, const float3 &u, const float &dt)
{
  // x and y
  s.p.x = s.p.x + (s.v*dt + 0.5f*u.x*dt*dt)*cos(s.theta + s.w*dt + 0.5f*u.y*dt*dt);
  s.p.y = s.p.y + (s.v*dt + 0.5f*u.x*dt*dt)*sin(s.theta + s.w*dt + 0.5f*u.y*dt*dt);

  //s and theta
  s.s = s.s + s.v*dt + 0.5f*u.x*dt*dt;
  s.theta = s.theta + s.w*dt + 0.5f*u.y*dt*dt;

  //v and w
  s.v = s.v + u.x*dt;
  s.w = s.w + u.y*dt;
}

__host__ __device__ __forceinline__
float3 dp_control(const State &s, const float3 &site, VoidPtrCarrier &aux_data, const UniformBinCarrier &ubc)
{
  // Cast all the needed pointers
  CUDA_MAT::Mat4Act *S_A = static_cast<CUDA_MAT::Mat4Act*>(aux_data[0]);
  CUDA_MAT::Vecf *bin_p = static_cast<CUDA_MAT::Vecf*>(aux_data[1]);
  CUDA_MAT::Vecf *bin_v = static_cast<CUDA_MAT::Vecf*>(aux_data[2]);
  CUDA_MAT::Vecf *bin_theta = static_cast<CUDA_MAT::Vecf*>(aux_data[3]);
  CUDA_MAT::Vecf *bin_w = static_cast<CUDA_MAT::Vecf*>(aux_data[4]);

  // Construct the relative state
  float s_relative[4];
  s_relative[0] = s.s - site.x; // relative station
  s_relative[1] = s.v; // relative velocity
  s_relative[2] = s.theta - site.y; // relative velocity
  s_relative[3] = s.w; //relative angular speed

  //dp_action u = CUDA_MAT::get_control(s_relative, *S_A, *bin_p, *bin_v, *bin_theta, *bin_w);
  dp_action u = CUDA_MAT::get_control_uniform_bin(s_relative, *S_A, ubc);
  // Return the control
  return make_float3(0,0,0.0f);
}

__host__ __device__ __forceinline__
float process_cost(const State &s, const State &goal, const EDTMap &map)
{
  float cost = 0;
  float2 dist_err = s.p - goal.p;
  cost += 0.5f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + 0.2f*sqrt(s.v*s.v + s.w*s.w);

  int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
  int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
  int iz = floor( (0.35 - map.m_origin.z) / map.m_grid_step + 0.5);

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

  if (rd < 1.0)
    cost += 100;
#endif

  if (sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) > 4)
  {
    cost += 0.5f*M_PI;
  }
  else
  {
    float yaw_diff = s.theta - goal.theta;
    yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
    cost += 0.5f*fabs(yaw_diff);
  }

  return  cost;
}

__host__ __device__ __forceinline__
float final_cost(const State &s, const State &goal, const EDTMap &map)
{
  float cost = 0;
  float2 dist_err = s.p - goal.p;
  cost += 4.0f*sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) + 0.2f*sqrt(s.v*s.v + s.w*s.w);

  int ix = floor( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5);
  int iy = floor( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5);
  int iz = floor( (0.35 - map.m_origin.z) / map.m_grid_step + 0.5);

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

  if (rd < 1.0)
    cost += 100;
#endif

  if (sqrt(dist_err.x*dist_err.x + dist_err.y*dist_err.y) > 4)
  {
    cost += 4*M_PI;
  }
  else
  {
    float yaw_diff = s.theta - goal.theta;
    yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
    cost += 4*fabs(yaw_diff);
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
  }
}

//---
__host__ __device__ __forceinline__
void bound_ptcl_location(Particle &p, const State &s)
{
  for (int n = 0; n < PSO_STEPS; n++)
  {
    bound_between(p.curr_loc.site[n].x,  s.s-9.0f,  s.s+9.0f);
    bound_between(p.curr_loc.site[n].y,  s.theta-3, s.theta+3);
  }
}

//---
__host__ __device__ __forceinline__
void initialize_a_particle(const State &s, Particle &p)
{
  for (int i=0; i< PSO_STEPS; i++)
  {
    p.curr_loc.site[i].x = rand_float_gen(&(p.rs), s.s-6, s.s+6); // station target
    p.curr_loc.site[i].y = rand_float_gen(&(p.rs), s.theta-3, s.theta+3); // theta target
    p.curr_loc.site[i].z = 0;

    p.ptcl_vel.site[i].x = rand_float_gen(&(p.rs), -1.0f, 1.0f);
    p.ptcl_vel.site[i].y = rand_float_gen(&(p.rs), -1.0f, 1.0f);
    p.ptcl_vel.site[i].z = 0;
  }
  bound_ptcl_velocity(p);
  bound_ptcl_location(p, s);

  p.best_loc = p.curr_loc;
  //printf("%f\n",p.best_loc[0].x);
}
}
#endif // PSO_DYNAMICS_CUH
