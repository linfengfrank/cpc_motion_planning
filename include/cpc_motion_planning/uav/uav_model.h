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
    float yaw;
    __host__ __device__
    State():
      p(make_float3(0,0,0)),
      v(make_float3(0,0,0)),
      a(make_float3(0,0,0)),
      yaw(0)
    {
    }
  };

  struct Input
  {
    float jerk;
    __host__ __device__
    Input():jerk(0)
    {}
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

  State m_s_ini;

};
}

#endif // UAV_MODEL_H
