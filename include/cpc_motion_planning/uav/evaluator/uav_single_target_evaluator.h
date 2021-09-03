#ifndef UAV_SINGLE_TARGET_EVLUATOR_H
#define UAV_SINGLE_TARGET_EVLUATOR_H
#include <cpc_motion_planning/uav/model/uav_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
namespace UAV
{
class SingleTargetEvaluator
{
public:
  SingleTargetEvaluator():m_stuck(false),m_in_air(false),m_consider_fov(false)
  {
    m_carrot = make_float3(0,0,0);
    m_safety_radius = 0.41f;
    m_collision_radius = 0.31f;
  }

  ~SingleTargetEvaluator()
  {

  }

  __host__ __device__
  void evaluate_state(const UAVModel::State &s, const EDTMap &map, float &cost, float &d_edt) const
  {
    cost = 0;
    d_edt = 1000;
    // calculate the distance to carrot
    float3 dist_err = s.p - m_carrot;

    // -------------- Control Cost -------------- //
    // add the control cost
    cost += 0.3f*sqrtf(s.a.x*s.a.x + s.a.y*s.a.y + s.a.z*s.a.z);

    if (m_in_air)
    {
      // -------------- Navigation Cost -------------- //
      // if obstacle avoidance is activated
      if (m_stuck)
      {
        // stucked, try random move with a small speed in the horizontal direction
        cost += fabsf(sqrtf(s.v.x*s.v.x + s.v.y*s.v.y) - 0.3f);
        cost += 0.5f*3.0f*sqrtf(dist_err.z*dist_err.z);
      }
      else
      {
        // add the target cost
        cost += 0.5f*sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y + 3*dist_err.z*dist_err.z);

        // add the line following cost
        int3 line_map_crd = m_line_map.pos2coord(s.p);
  #ifdef  __CUDA_ARCH__
        if (!m_line_map.isInside(line_map_crd))
        {
          cost += 100;
        }
        else
        {
          float d2line = m_line_map.edt_const_at(line_map_crd).d*m_line_map.m_grid_step;
          cost += 5*d2line;
        }
  #endif
      }

      // -------------- Obstacle Cost -------------- //
      // add this cost to prevent it go outside the map area
      // it also returns as accessing outside the local map cause error
      int3 obs_map_crd = map.pos2coord(s.p);
      if (!map.isInside(obs_map_crd))
      {
        cost += 100;
        return;
      }

      // add obstacle cost
#ifdef  __CUDA_ARCH__
      // get the minimum distance to obstacle
      d_edt = map.edt_const_at(obs_map_crd).d*map.m_grid_step;

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
        if (!map.edt_const_at(obs_map_crd).s)
          cost += 100;
#endif
      }
    }
    else
    {
      // -------------- Navigation Cost (not in air) -------------- //
      // when the obstacle aovidance is not activate, just penalize the euclidean
      // distance to the carrot's position
      cost += 0.5f*sqrtf(dist_err.x*dist_err.x + dist_err.y*dist_err.y + 3*dist_err.z*dist_err.z);
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
    // d_edt is the edt distance at this poisition
    float cost,d_edt;
    evaluate_state(s, map, cost, d_edt);

    return  cost;
  }

  bool m_stuck;
  bool m_in_air;
  bool m_consider_fov;
  float m_curr_yaw;
  float3 m_curr_pos;
  float3 m_carrot;
  EDTMap m_line_map;
  float m_safety_radius;
  float m_collision_radius;
};
}

#endif // UAV_SINGLE_TARGET_EVLUATOR_H
