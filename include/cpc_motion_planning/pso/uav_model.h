#ifndef UAV_MODEL_H
#define UAV_MODEL_H
#include <curand_kernel.h>
#include <cpc_motion_planning/pso/pso_dynamics.cuh>
namespace PSO
{
class UAVModel
{
public:
UAVModel()
{

}

~UAVModel()
{

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

};
}

#endif // UAV_MODEL_H
