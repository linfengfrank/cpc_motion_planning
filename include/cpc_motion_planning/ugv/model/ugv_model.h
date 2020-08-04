#ifndef UGV_MODEL_H
#define UGV_MODEL_H
#include <curand_kernel.h>
#include <cpc_motion_planning/pso/pso_utilities.cuh>

namespace UGV
{
class UGVModel
{
public:
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

  struct Input
  {
    float acc;
    float alpha;
    __host__ __device__
    Input():acc(0),alpha(0)
    {}
  };

  UGVModel()
  {

  }

  ~UGVModel()
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

  __host__ __device__
  void model_forward_with_slip(State &s, const float3 &u, const float &dt, const float slip = 1.0f)
  {
    // x and y
    s.p.x = s.p.x + (s.v*dt + 0.5f*u.x*dt*dt)*cos(s.theta + s.w*dt*slip + 0.5f*u.y*dt*dt);
    s.p.y = s.p.y + (s.v*dt + 0.5f*u.x*dt*dt)*sin(s.theta + s.w*dt*slip + 0.5f*u.y*dt*dt);

    //s and theta
    s.s = s.s + s.v*dt + 0.5f*u.x*dt*dt;
    s.theta = s.theta + s.w*dt*slip + 0.5f*u.y*dt*dt;

    //v and w
    s.v = s.v + u.x*dt;
    s.w = s.w + u.y*dt;
  }
  State m_s_ini;

};
}

#endif // UGV_MODEL_H
