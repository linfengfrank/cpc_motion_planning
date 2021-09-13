#ifndef UGV_NF1_EVALUATOR_H
#define UGV_NF1_EVALUATOR_H

#include <cpc_motion_planning/ugv/model/ugv_model.h>
#include <cpc_motion_planning/dynamic_programming.cuh>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/ugv/evaluator/bilinear_interpolation.h>

namespace UGV
{
class NF1Evaluator
{
public:

  struct Target
  {
    UGVModel::State s;
    int path_id;
    int act_id;
    float reaching_radius;
    Target():path_id(-1),act_id(-1),reaching_radius(1.0f)
    {

    }
  };

  NF1Evaluator()
  {
    m_accurate_reaching = false;
    is_forward = true;
    m_pure_turning = false;
    m_nf1_received = false;
    m_stuck = false;
    m_using_auto_direction = false;
    m_safety_radius = 0.401f;
    m_max_speed = 1.0f;
  }

  ~NF1Evaluator()
  {

  }

  __host__ __device__
  void setTarget(const Target &goal)
  {
    m_goal = goal;
  }

  __host__ __device__
  void calculate_bounding_centres(const UGVModel::State &s, float2 &c_r, float2 &c_f) const
  {
    float2 uni_dir = make_float2(cosf(s.theta),sinf(s.theta));
    c_f = s.p + 0.25f*uni_dir;
    c_r = s.p - 0.15f*uni_dir;
  }

  __host__ __device__
  float getEDT(const float2 &p, const EDTMap &map) const
  {
    int ix = floorf( (p.x - map.m_origin.x) / map.m_grid_step + 0.5f);
    int iy = floorf( (p.y - map.m_origin.y) / map.m_grid_step + 0.5f);
    int iz = floorf( (0 - map.m_origin.z) / map.m_grid_step + 0.5f);
    if (ix<0 || ix>=map.m_map_size.x ||
        iy<0 || iy>=map.m_map_size.y ||
        iz<0 || iz>=map.m_map_size.z)
    {
      return 0;
    }
    else
    {
#ifdef  __CUDA_ARCH__
      return map.edt_const_at(ix,iy,iz).d * map.m_grid_step;
#else
      return 0;
#endif
    }
  }

  __host__ __device__
  float getMinDist(const UGVModel::State &s, const EDTMap &map) const
  {
    float2 c_f,c_r;
    calculate_bounding_centres(s, c_r, c_f);
    return min(getEDT(c_r,map),getEDT(c_f,map));
  }

  __host__ __device__
  float getDesiredHeading(const CUDA_GEO::coord &c) const
  {
#ifdef  __CUDA_ARCH__
    return m_nf1_map.theta_const_at(c.x,c.y,0);
#else
    return 0;
#endif
  }

  __host__ __device__
  float2 get_head_pos(const UGVModel::State &s,float r) const
  {
    float2 head_pos;
    head_pos.x = s.p.x + r*cosf(s.theta);
    head_pos.y = s.p.y + r*sinf(s.theta);
    return head_pos;
  }

  __host__ __device__
  float calculate_nf1_cost(const UGVModel::State &s, float head_dist) const
  {
    float nf_cost = 0;
    float2 head_pos;
    if (is_forward)
      head_pos = get_head_pos(s,head_dist);
    else
      head_pos = get_head_pos(s,-head_dist);

    // Head cost is needed anyway
    CUDA_GEO::coord head_c = m_nf1_map.pos2coord(make_float3(head_pos.x,head_pos.y,0));

    // Calculate the distance between vehicle center and final goal
    float2 ctr_diff_to_goal = s.p-m_goal.s.p;
    float ctr_dist_to_goal = sqrtf(dot(ctr_diff_to_goal,ctr_diff_to_goal));
    if(ctr_dist_to_goal > fabsf(head_dist) || !m_accurate_reaching)
    {
      // If distance too far, only consider the Head cost
#ifdef  __CUDA_ARCH__
      nf_cost = m_nf1_map.nf1_const_at(head_c.x,head_c.y,0)+5;
#endif
    }
    else
    {
      // If close to the final goal, also consider the center cost
      float k = ctr_dist_to_goal / fabsf(head_dist);
      CUDA_GEO::coord ctr_c = m_nf1_map.pos2coord(make_float3(s.p.x,s.p.y,0));
#ifdef  __CUDA_ARCH__
      // Must use c.x c.y and 0 here! Because the NF1 map has only 1 layer.
      nf_cost = k*(m_nf1_map.nf1_const_at(head_c.x,head_c.y,0)+5) + (1.0f - k) * m_nf1_map.nf1_const_at(ctr_c.x,ctr_c.y,0);
#endif
    }

    return nf_cost;
  }

  __host__ __device__
  float process_cost(const UGVModel::State &s, const EDTMap &map, const float &time, PSO::EvaData &data) const
  {
    float cost = 0;

    if (m_using_auto_direction)
      is_forward = data.is_forward;

    float rd = getMinDist(s,map);
    //cost += expf(-9.5f*rd)*10;

    if (rd < m_safety_radius)
      cost += (1000 + expf(-4.5f*rd)*1000);

    if (rd < PSO::MIN_DIST && time < 0.31f)
    {
      data.collision = true;
    }

    data.min_dist = rd;

//    if (fabsf(s.v) > m_max_speed)
//      cost += 100*(fabsf(s.v) - m_max_speed);

    if (m_nf1_received)
    {
      if (m_pure_turning)
      {
        //Pure heading cost
        cost += 5.0f*sqrtf(s.v*s.v); // stay still during turning
        float yaw_diff = s.theta - m_goal.s.theta;
        yaw_diff = yaw_diff - floorf((yaw_diff + CUDA_F_PI) / (2 * CUDA_F_PI)) * 2 * CUDA_F_PI;
        cost += 0.5f*fabsf(yaw_diff);
      }
      else
      {
        //either stuck or normal mode
        CUDA_GEO::coord c = m_nf1_map.pos2coord(make_float3(s.p.x,s.p.y,0));
        float nf_cost = calculate_nf1_cost(s, 0.3f);
        if (m_stuck)
        {
          //stuck mode, encourage random move to get out of stuck
//          cost += expf(-4.5f*rd)*100;
          cost += 2.0f*fabsf(fabsf(s.v) - 0.3f) + 0.5f*fabsf(fabsf(s.w) - 0.2f);// + 0.02f*nf_cost;
        }
        else
        {
          //normal mode
          cost += 0.4f*nf_cost+0.25f*s.v*s.v + 1.0f*fabsf(s.w)*fabsf(s.v);//1.0f*sqrtf(0.005f*s.v*s.v + 1.0f*s.w*s.w*gain);

          //float yaw_diff;
          //if (is_forward)
          //  yaw_diff = s.theta - getDesiredHeading(c);//bilinear_theta(s.p, m_nf1_map);//getDesiredHeading(c);
          //else
          //  yaw_diff = s.theta + CUDA_F_PI - getDesiredHeading(c);

          //yaw_diff = yaw_diff - floorf((yaw_diff + CUDA_F_PI) / (2 * CUDA_F_PI)) * 2 * CUDA_F_PI;
          //cost += yaw_diff*yaw_diff*s.v*s.v;
        }
      }
    }
    else
    {
      // have not received the guidance function map yet, stay still
      cost += 1.0f*sqrtf(s.v*s.v + s.w*s.w);
    }

    return  cost;
  }

  __host__ __device__
  float final_cost(const UGVModel::State &s, const EDTMap &map) const
  {
    float cost = 0;

    if(!m_pure_turning && m_nf1_received && !m_stuck)
    {
      cost += 20.0f * calculate_nf1_cost(s,0.3f);
    }

    return  cost;
  }

  bool m_accurate_reaching;
  Target m_goal;
  mutable bool is_forward;
  bool m_pure_turning;
  bool m_stuck;
  NF1MapDT m_nf1_map;
  bool m_nf1_received;
  bool m_using_auto_direction;
  float m_safety_radius;
  float m_max_speed;
};
}

#endif // UGV_NF1_EVALUATOR_H
