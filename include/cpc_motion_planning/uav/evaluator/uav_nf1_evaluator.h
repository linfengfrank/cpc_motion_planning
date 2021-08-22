#ifndef UAV_NF1_EVLUATOR_H
#define UAV_NF1_EVLUATOR_H
#include <cpc_motion_planning/uav/model/uav_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_nf1map.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
namespace UAV
{
class NF1Evaluator
{
public:

  struct Target
  {
    bool oa;
  };

  NF1Evaluator():m_stuck(false),m_in_air(false),m_consider_fov(false)
  {
    m_carrot = make_float3(0,0,0);
    m_safety_radius = 0.41f;
    m_collision_radius = 0.31f;
  }

  ~NF1Evaluator()
  {

  }

  __host__ __device__
  void evaluate_state(const UAVModel::State &s, const EDTMap &map, float &cost, float &d_edt) const
  {
    cost = 0;
    d_edt = 1000;
    // calculate the distance to carrot
    float3 dist_err = s.p - m_carrot;

    // -------------- Height Cost -------------- //
    // add in the height difference cost
    // The disred fly height is determined by the mid planner and passed in through m_carrot
    cost += 0.5f*3.0f*sqrtf(dist_err.z*dist_err.z);

    // -------------- Control Cost -------------- //
    // add the control cost
    cost += 0.1f*sqrtf(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

    if (m_in_air)
    {
      // -------------- Navigation Cost -------------- //
      // if obstacle avoidance is activated
      CUDA_GEO::coord c = m_nf1_map.pos2coord(s.p);
      if (m_stuck)
      {
        // stucked, try random move with a small speed
        cost += fabsf(sqrtf(s.v.x*s.v.x + s.v.y*s.v.y) - 0.3f);
      }
      else
      {
        // add the NF1 cost generated by the mid planner  and passed in through the m_nf1_map
#ifdef  __CUDA_ARCH__
        cost += 0.5f*m_nf1_map.nf1_const_at(c.x,c.y,0);
#endif
      }

      // -------------- Obstacle Cost -------------- //
      // add this cost to prevent it go outside the map area
      // it also returns as accessing outside the local map cause error
      int ix = floorf( (s.p.x - map.m_origin.x) / map.m_grid_step + 0.5f);
      int iy = floorf( (s.p.y - map.m_origin.y) / map.m_grid_step + 0.5f);
      int iz = floorf( (s.p.z - map.m_origin.z) / map.m_grid_step + 0.5f);

      // If we are using a 2D map, set the height coord to ZERO
      // since there is only one layer
      if (map.m_map_size.z == 1)
        iz = 0;

      if (ix<0 || ix>=map.m_map_size.x ||
          iy<0 || iy>=map.m_map_size.y ||
          iz<0 || iz>=map.m_map_size.z)
      {
        cost += 100;
        return;
      }

      // add obstacle cost
#ifdef  __CUDA_ARCH__
      // get the minimum distance to obstacle
      d_edt = map.edt_const_at(ix,iy,iz).d*map.m_grid_step;

      // add obstacle based on EDT distance
      float free_dist = fmaxf(d_edt - m_safety_radius, 0);
      cost += expf(-9.5f*free_dist)*10;

      if (d_edt < m_safety_radius)
        cost += (1000 + expf(-4.5f*d_edt)*1000);

#endif
      // FOV cost, limit the planned trajectory inside the sensor's cone
      if (m_consider_fov)
      {
        // -------------- FOV Cost -------------- //
        float3 diff = s.p - m_curr_pos;
        diff.z = 0;
        if (sqrtf(diff.x*diff.x + diff.y*diff.y) > 0.3f)
        {
          float theta = atan2f(diff.y,diff.x);
          theta -= m_curr_yaw;
          theta = theta - floorf((theta + M_PI) / (2 * M_PI)) * 2 * M_PI;
          if (fabsf(theta) > M_PI*0.25f)
            cost += 100;
        }
        // -------------- Seen Map Cost -------------- //
#ifdef  __CUDA_ARCH__
        // add this cost to avoid unoberserved volume
        // only activate when m_consider_fov is set
        if (!map.edt_const_at(ix,iy,iz).s)
          cost += 100;
#endif
      }
    }
    else
    {
      // -------------- Navigation Cost (not in air) -------------- //
      // when the obstacle aovidance is not activate, just penalize the euclidean
      // distance to the carrot's position
      cost += 0.5f*1.0f*sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y);
    }
  }

  __host__ __device__
  float process_cost(const UAVModel::State &s, const EDTMap &map, const float &time, bool &collision) const
  {
    // cost is the objective cost
    // d_edt is the edt distance at this poisition
    float cost,d_edt;
    evaluate_state(s, map, cost, d_edt);

    // when it is too close to the obstacle, while time is too close set the collision flag
    if (d_edt < m_collision_radius && time < 1.5f)
    {
      collision = true;
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UAVModel::State &s, const EDTMap &map) const
  {
    // cost is the objective cost
    float cost = 0;
    if (m_in_air && !m_stuck)
    {
      // -------------- Navigation Cost -------------- //
      // add the NF1 cost generated by the mid planner and passed in through the m_nf1_map
      CUDA_GEO::coord c = m_nf1_map.pos2coord(s.p);
#ifdef  __CUDA_ARCH__
      cost += 2*m_nf1_map.nf1_const_at(c.x,c.y,0);
#endif
    }
    return cost;
  }

  bool m_stuck;
  bool m_in_air;
  bool m_consider_fov;
  float m_curr_yaw;
  float3 m_curr_pos;
  float3 m_carrot;
  NF1Map m_nf1_map;
  float m_safety_radius;
  float m_collision_radius;
};
}
#endif // UAV_NF1_EVLUATOR_H
