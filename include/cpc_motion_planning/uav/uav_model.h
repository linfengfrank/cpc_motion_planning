#ifndef UAV_MODEL_H
#define UAV_MODEL_H
#include <curand_kernel.h>
#include <cpc_motion_planning/pso/pso_utilities.cuh>
namespace UAV
{
class UAVModel
{
public:
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

  UAVModel()
  {

  }

  ~UAVModel()
  {

  }

  __host__ __device__
  void set_ini_state(const State &s)
  {
    m_s_ini = s;
  }

  __host__ __device__
  State get_ini_state()
  {
    return m_s_ini;
  }

  __host__ __device__
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

  //---
  __host__ __device__
  void bound_ptcl_velocity(PSO::Particle &p)
  {
    for (int n = 0; n < PSO::PSO_STEPS; n++)
    {
      PSO::bound_between(p.ptcl_vel.site[n].x, -0.5f, 0.5f);
      PSO::bound_between(p.ptcl_vel.site[n].y, -0.5f, 0.5f);
      PSO::bound_between(p.ptcl_vel.site[n].z, -0.5f, 0.5f);
    }
  }

  //---
  __host__ __device__
  void bound_ptcl_location(PSO::Particle &p)
  {
    for (int n = 0; n < PSO::PSO_STEPS; n++)
    {
      PSO::bound_between(p.curr_loc.site[n].x,  m_s_ini.p.x-9.0f,  m_s_ini.p.x+9.0f);
      PSO::bound_between(p.curr_loc.site[n].y,  m_s_ini.p.y-9.0f,  m_s_ini.p.y+9.0f);
      PSO::bound_between(p.curr_loc.site[n].z,  m_s_ini.p.z-9.0f,  m_s_ini.p.z+9.0f);
      //bound_between(p.curr_loc.site[n].z,  1.6f,  1.65f);
    }
  }

  //---
  __host__ __device__
  void initialize_a_particle(PSO::Particle &p)
  {
    for (int i=0; i< PSO::PSO_STEPS; i++)
    {
      p.curr_loc.site[i].x = PSO::rand_float_gen(&(p.rs), m_s_ini.p.x-6, m_s_ini.p.x+6); // station target
      p.curr_loc.site[i].y = PSO::rand_float_gen(&(p.rs), m_s_ini.p.y-6, m_s_ini.p.y+6); // station target
      p.curr_loc.site[i].z = PSO::rand_float_gen(&(p.rs), m_s_ini.p.z-6, m_s_ini.p.z+6); // station target

      p.ptcl_vel.site[i].x = PSO::rand_float_gen(&(p.rs), -1.0f, 1.0f);
      p.ptcl_vel.site[i].y = PSO::rand_float_gen(&(p.rs), -1.0f, 1.0f);
      p.ptcl_vel.site[i].z = PSO::rand_float_gen(&(p.rs), -1.0f, 1.0f);
    }
    bound_ptcl_velocity(p);
    bound_ptcl_location(p);

    p.best_loc = p.curr_loc;
    //printf("%f\n",p.best_loc[0].x);
  }

  State m_s_ini;

};
}

#endif // UAV_MODEL_H
