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

private:
  // Distance from c0 to line_seg c1_c2
  inline float point2lineDist(const float2 & c1, const float2 & c2, const float2 & c0)
  {
    float2 a = c1-c0;
    float2 b = c2-c1;


    float a_dot_b = dot(a,b);

    if (dot(b,b) < 1e-6)
      return sqrtf(static_cast<float>(dot(a,a)));

    return sqrtf((dot(a,a)*dot(b,b) - a_dot_b*a_dot_b)/dot(b,b));
  }

  //----
  std::vector<size_t> findSplitCoords(const std::vector<float2> &path, float split_dist)
  {
    std::vector<size_t> split_pts;

    // If there is no path, return empty
    if (path.size() == 0)
    {
      return split_pts;
    }

    std::stack<std::pair<unsigned int,unsigned int>> pls; // fist is anchor, second is target
    pls.push(std::make_pair(path.size() - 1, 0));

    unsigned int check = static_cast<unsigned int>(path.size()) - 1;

    while (pls.size() > 0)
    {
      unsigned int target = pls.top().second;
      unsigned int anchor = pls.top().first;
      pls.pop();
      // find the largest distance
      unsigned int max_id = 0;
      float max_dist = 0;
      for (unsigned int j = target; j< anchor; j++)
      {
        float dist = point2lineDist(path[anchor],path[target],path[j]);
        if (dist > max_dist)
        {
          max_dist = dist;
          max_id = j;
        }
      }

      if (max_dist > split_dist)
      {
        pls.push(std::make_pair(max_id, target));
        pls.push(std::make_pair(anchor, max_id));
        target = max_id;
      }
      else
      {
        split_pts.push_back(target);
        if (target >= check)
        {
          ROS_ERROR("Wrong split sequence");
          exit(-1);
        }
        check = target;
      }
    }
    std::reverse(std::begin(split_pts), std::end(split_pts));
    return split_pts;
  }
};

#endif // UGV_REFTRAJ_LOCAL_PLANNER_H
