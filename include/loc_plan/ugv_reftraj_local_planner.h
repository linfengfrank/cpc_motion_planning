#ifndef UGV_REFTRAJ_LOCAL_PLANNER_H
#define UGV_REFTRAJ_LOCAL_PLANNER_H

#include <loc_plan/ugv_base_local_planner.h>
#include <visualization_msgs/Marker.h>

#define REF_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::RefTrajEvaluator,UGV::UGVSwarm<3>

class UGVRefTrajMotionPlanner : public UGVLocalMotionPlanner
{
public:
  UGVRefTrajMotionPlanner();
  ~UGVRefTrajMotionPlanner();

protected:
  virtual void do_start();
  virtual void do_normal();
  virtual void do_stuck();
  virtual void do_emergent();
  virtual void do_braking();
  virtual void do_pos_reached();
  virtual void do_fully_reached();

private:
  void plan_call_back(const ros::TimerEvent&);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void cycle_init();
  void load_ref_lines();
  void calculate_ref_traj(float3 vehicle_config);
  void linecirc_inter_dist(const float3 &seg_a, const float3 &seg_b, const float3 &circ_pos, float3 &closest, float &dist_v_len) const;
  float3 calculate_unit_vector(const float3 &seg_a, const float3 &seg_b);
  float calculate_length(const float3 &seg_a, const float3 &seg_b);
  UGV::UGVModel::State float3_to_goal_state(const float3 &in)
  {
    UGV::UGVModel::State tmp_goal;
    tmp_goal.p = make_float2(in.x,in.y);
    tmp_goal.theta = in.z;
    return tmp_goal;
  }

  std::vector<float3> interpol(const float3 &a, const float3 &b, int n = -1)
  {
    if (n == -1)
    {
      n = static_cast<int>(sqrtf(dot(b-a,b-a))/0.1f);
    }
    n = max(2,n);
    std::vector<float3> tmp(n);
    float3 delta = (b-a)/static_cast<float>(n-1);
    for (int i = 0; i < n; i++)
    {
      tmp[i] = a + delta*static_cast<float>(i);
    }
    return tmp;
  }

  std::vector<float> interpol(const float &a, const float &b, int n = -1)
  {
    if (n == -1)
    {
      n = static_cast<int>(fabsf(b-a)/0.1f);
    }
    n = max(2,n);
    std::vector<float> tmp(n);
    float delta = (b-a)/static_cast<float>(n-1);
    for (int i = 0; i < n; i++)
    {
      tmp[i] = a + delta*static_cast<float>(i);
    }
    return tmp;
  }

  float in_pi(float in)
  {
    return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
  }

  float un_in_pi(float in, float last)
  {
    return in_pi(in-last) + last;
  }

  std::vector< float > interp1( std::vector< float > &x, std::vector< float > &y, std::vector< float > &x_new )
  {
      std::vector< float > y_new;
      y_new.reserve( x_new.size() );

//      std::vector< float > dx, dy, slope, intercept;
//      dx.reserve( x.size() );
//      dy.reserve( x.size() );
//      slope.reserve( x.size() );
//      intercept.reserve( x.size() );
//      for( int i = 0; i < x.size(); ++i ){
//          if( i < x.size()-1 )
//          {
//              dx.push_back( x[i+1] - x[i] );
//              dy.push_back( y[i+1] - y[i] );
//              slope.push_back( dy[i] / dx[i] );
//              intercept.push_back( y[i] - x[i] * slope[i] );
//          }
//          else
//          {
//              dx.push_back( dx[i-1] );
//              dy.push_back( dy[i-1] );
//              slope.push_back( slope[i-1] );
//              intercept.push_back( intercept[i-1] );
//          }
//      }

      for ( int i = 0; i < x_new.size(); ++i )
      {
          int idx = findNearestNeighbourIndex( x_new[i], x );
          y_new.push_back( y[idx] );

      }
      return y_new;
  }

  int findNearestNeighbourIndex( float value, std::vector< float > &x )
  {
      float dist = FLT_MAX;
      int idx = -1;
      for ( int i = 0; i < x.size(); ++i ) {
          float newDist = value - x[i];
          if ( newDist > 0 && newDist < dist ) {
              dist = newDist;
              idx = i;
          }
      }

      return idx;
  }

private:
  ros::Subscriber m_goal_sub;
  ros::Publisher m_vis_pub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  bool m_goal_received;
  PSO::Planner<REF_UGV> *m_pso_planner;
  UGV::RefTrajEvaluator::Ref m_ref_traj;
  float m_ref_v, m_ref_w;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;
  std::vector<UGV::UGVModel::State> m_traj;
  bool cycle_initialized;
  int m_braking_start_cycle;
  std::vector<float3> m_path;
  std::vector<float3> m_path_len;
};

#endif // UGV_REFTRAJ_LOCAL_PLANNER_H
