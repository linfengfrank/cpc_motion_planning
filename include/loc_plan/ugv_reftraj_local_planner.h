#ifndef UGV_REFTRAJ_LOCAL_PLANNER_H
#define UGV_REFTRAJ_LOCAL_PLANNER_H

#include <loc_plan/ugv_base_local_planner.h>
#include <visualization_msgs/Marker.h>

#define REF_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::RefTrajEvaluator,UGV::UGVSwarm<3>

class UGVRefTrajMotionPlanner : public UGVLocalMotionPlanner
{
public:
  struct line_seg
  {
    float2 a;
    float2 b;
    float2 uni;
    float tht;
    float dist;
    line_seg(float2 a_, float2 b_):a(a_),b(b_)
    {
      dist = sqrtf(dot(b-a,b-a));

      if (dist > 1e-6)
        uni = (b-a)/dist;
      else
        uni = make_float2(0,0);

      tht = atan2(uni.y,uni.x);
    }
  };

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
  void calculate_ref_traj(float2 vehicle_pos);
//  void linecirc_inter_dist(const float3 &seg_a, const float3 &seg_b, const float3 &circ_pos, float3 &closest, float &dist_v_len) const;
//  float3 calculate_unit_vector(const float3 &seg_a, const float3 &seg_b);
//  float calculate_length(const float3 &seg_a, const float3 &seg_b);
//  UGV::UGVModel::State float3_to_goal_state(const float3 &in)
//  {
//    UGV::UGVModel::State tmp_goal;
//    tmp_goal.p = make_float2(in.x,in.y);
//    tmp_goal.theta = in.z;
//    return tmp_goal;
//  }

  std::vector<float2> interpol(const float2 &a, const float2 &b, float v = 1.0f)
  {
    int n = static_cast<int>(sqrtf(dot(b-a,b-a))/(v*PSO::PSO_SIM_DT));
    n = max(2,n);
    std::vector<float2> tmp(n);
    float2 delta = (b-a)/static_cast<float>(n-1);
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

private:
  ros::Subscriber m_goal_sub;
  ros::Publisher m_vis_pub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  bool m_goal_received;
  PSO::Planner<REF_UGV> *m_pso_planner;
  float m_ref_v, m_ref_w;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;
  std::vector<UGV::UGVModel::State> m_traj;
  bool cycle_initialized;
  int m_braking_start_cycle;
  std::vector<std::vector<line_seg>> m_line_list;
  std::vector<std::vector<float2>> m_path_list;
  UGV::RefTrajEvaluator::Target m_tgt;
  size_t m_path_idx;
};

#endif // UGV_REFTRAJ_LOCAL_PLANNER_H
