#include <cpc_motion_planning/uav/uav_repulsive_field.h>

namespace UAV
{
__device__
bool find_obs_pnt(const EDTMap &map, float3 dir, const float3 &p_ini, float3 &obs_pnt)
{
  dir = dir/sqrtf(dot(dir,dir));
  float step = map.m_grid_step * 0.5f;
  int ix, iy, iz;
  bool stop = false;
  for (float i=0; i<3.0f; i+=step)
  {
    obs_pnt = p_ini + dir*i;

    ix = floorf( (obs_pnt.x - map.m_origin.x) / map.m_grid_step + 0.5f);
    iy = floorf( (obs_pnt.y - map.m_origin.y) / map.m_grid_step + 0.5f);
    iz = floorf( (obs_pnt.z - map.m_origin.z) / map.m_grid_step + 0.5f);

    if (ix<0 || ix>=map.m_map_size.x ||
        iy<0 || iy>=map.m_map_size.y ||
        iz<0 || iz>=map.m_map_size.z)
    {
      stop = true;
      break;
    }

    if(map.edt_const_at(ix,iy,iz).d*map.m_grid_step <= 0.05f)
    {
      stop = true;
      break;
    }
  }
  return stop;
}

__device__
float3 calc_single_repulsive_force(const UAV::UAVModel::State &s, const float3 &o)
{
  float3 p = s.p;
  float3 v = s.v;

  float r = sqrtf(dot(o-p,o-p));
  float3 n = o-p;
  float3 n_hat = n/r;

  float vn = dot(v,n_hat);
  if (vn < 0)
      vn = 0;

  float3 F = make_float3(0,0,0);
  F += 1.0f/(r*r)*(-n_hat);
  F += 1.0f/(r*r)*vn*(-n_hat);
  F += 1.0f/(r*r)*vn*(vn*(-n_hat)-v);

  return F;

}
//---
__global__
void calc_repulsive_forces_kernel(EDTMap map,RepulseRegister* rgs, UAV::UAVModel::State s, unsigned int N)
{
  int idx = threadIdx.x+blockDim.x*blockIdx.x;
  float theta = static_cast<float>(idx)/static_cast<float>(N)*2.0f*3.1415926f;
  float3 dir=make_float3(cosf(theta),sinf(theta),0);

  float3 obs_pnt = make_float3(0,0,0);
  if (find_obs_pnt(map, dir, s.p, obs_pnt))
  {
    if (sqrtf(dot(obs_pnt-s.p,obs_pnt-s.p)) > 0.3f)
    {
      rgs[idx].force = calc_single_repulsive_force(s, obs_pnt);
      rgs[idx].valid = true;
    }
    else
    {
      rgs[idx].force = make_float3(0,0,0);
      rgs[idx].valid = false;
    }
  }
  else
  {
    rgs[idx].force = make_float3(0,0,0);
    rgs[idx].valid = false;
  }
}
//---
void calc_repulsive_forces(const EDTMap &map, RepulseRegister *rgs, const UAV::UAVModel::State &s, unsigned int N)
{
  calc_repulsive_forces_kernel<<<1,N>>>(map, rgs, s, N);
}
}

