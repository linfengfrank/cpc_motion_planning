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

private:
  ros::Subscriber m_goal_sub;
  ros::Publisher m_vis_pub;
  ros::ServiceClient m_astar_client;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  bool m_goal_received;
  PSO::Planner<REF_UGV> *m_pso_planner;
  float m_ref_v, m_ref_w, m_ref_theta;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt, m_tht_err_reset_ctt;
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
  //-----------------
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
  //-----------------
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

  //-----------------
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

  //-----------------
  void update_path(const std::vector<float2> &wps)
  {
    // Use split & merge to identify the wps of sharp turning
    std::set<size_t> split_sharp_wp_ids;
    std::vector<size_t> split_wp_ids = findSplitCoords(wps, 3.0f);
    for (size_t i = 1; i< split_wp_ids.size()-1; i++)
    {
      line_seg l1(wps[split_wp_ids[i-1]],wps[split_wp_ids[i]]);
      line_seg l2(wps[split_wp_ids[i]],wps[split_wp_ids[i+1]]);

      if (fabsf(in_pi(l1.tht-l2.tht)) > 0.25*M_PI)
      {
        split_sharp_wp_ids.insert(split_wp_ids[i]);
        //std::cout<<split_wp_ids[i]<<std::endl;
      }
    }

    // Construct the line list
    std::vector<line_seg> lines;
    for (size_t i = 0; i < wps.size()-1; i++)
    {
      lines.push_back(line_seg(wps[i],wps[i+1]));
    }

    m_line_list.clear();
    std::vector<line_seg> tmp;
    for (size_t i = 0; i < lines.size()-1; i++)
    {
      tmp.push_back(lines[i]);
      if (fabsf(in_pi(lines[i].tht-lines[i+1].tht)) > 0.25*M_PI || split_sharp_wp_ids.count(i+1) != 0)
      {
        m_line_list.push_back(tmp);
        tmp.clear();
      }
    }
    tmp.push_back((lines.back()));
    m_line_list.push_back(tmp);

    // Construct the path
    m_path_list.clear();
    for (size_t i = 0; i < m_line_list.size(); i++)
    {
      std::vector<float2> path;
      for (size_t j = 0; j < m_line_list[i].size(); j++)
      {
        std::vector<float2> tmp_pol = interpol(m_line_list[i][j].a,m_line_list[i][j].b,0.8f);
        for (float2 pol : tmp_pol)
        {
          path.push_back(pol);
        }
      }
      m_path_list.push_back(path);
    }

    m_path_idx = 0;
  }

  //-----------------
  void show_path(const std::vector<float2> &wps)
  {
    // For visulization
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    for (float2 p: wps)
    {
      geometry_msgs::Point pnt;
      pnt.x = p.x;
      pnt.y = p.y;
      pnt.z = 0;
      line_strip.points.push_back(pnt);
    }
    m_vis_pub.publish(line_strip);
  }
};

#endif // UGV_REFTRAJ_LOCAL_PLANNER_H
