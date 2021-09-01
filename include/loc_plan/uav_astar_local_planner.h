#ifndef UAV_ASTAR_MOTION_PLANNER_H
#define UAV_ASTAR_MOTION_PLANNER_H
#include <loc_plan/uav_base_local_planner.h>
#include <cpc_motion_planning/guide_line.h>
#define SIMPLE_UAV UAV::UAVModel,UAV::UAVDPControl,UAV::SingleTargetEvaluator,UAV::UAVSwarm<1>
#define EMERGENT_UAV UAV::UAVModel,UAV::UAVJLTControl,UAV::SingleTargetEvaluator,UAV::UAVSwarm<1>

class UAVAstarMotionPlanner : public UAVLocalMotionPlanner
{
public:
  UAVAstarMotionPlanner();
  ~UAVAstarMotionPlanner();

protected:
  virtual void do_at_ground();
  virtual void do_taking_off();
  virtual void do_in_air();
  virtual void do_stuck();
  virtual void do_emergent();
  virtual void do_braking();

private:
  void plan_call_back(const ros::TimerEvent&);
  void guide_line_call_back(const cpc_motion_planning::guide_line::ConstPtr &msg);
  void cycle_init();
  void set_init_state(const UAV::UAVModel::State& trans, const JLT::State &yaw);
  //---
  void set_planner_goal(UAV::SingleTargetEvaluator::Target goal, bool turn_on_obstacle_avoidance = true)
  {
    //set the goal with updated obstacle avoidance flag
    goal.oa = turn_on_obstacle_avoidance;
    m_pso_planner->m_eva.setTarget(goal);
    m_emergent_planner->m_eva.setTarget(goal);
  }
  //---
  float calculate_distance(const float3 &a, geometry_msgs::Point &b)
  {
    return sqrtf((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
  }
  //---
  inline float pnt2line_dist(const CUDA_GEO::pos & c1, const CUDA_GEO::pos & c2, const CUDA_GEO::pos & c0)
  {
    CUDA_GEO::pos a = c1-c0;
    CUDA_GEO::pos b = c2-c1;

    float a_square = a.square();
    float b_square = b.square();
    float a_dot_b = a.x*b.x + a.y*b.y + a.z*b.z;

    if (b_square < 1e-3)
      return sqrtf(static_cast<float>(a_square));

    return sqrtf(static_cast<float>(a_square*b_square - a_dot_b*a_dot_b)/static_cast<float>(b_square));
  }
  //---
  CUDA_GEO::pos Point2Pos(const geometry_msgs::Point &pnt)
  {
    CUDA_GEO::pos p(pnt.x, pnt.y, pnt.z);
    return p;
  }
  //---
  inline float3 Point2float3(const geometry_msgs::Point &pnt)
  {
    return make_float3(pnt.x, pnt.y, pnt.z);
  }
  //---
  bool is_curvature_too_big(const std::vector<geometry_msgs::Point> &path, size_t start, size_t end,float th_dist = 0.25f)
  {
    geometry_msgs::Point start_point = path[start];
    geometry_msgs::Point end_point = path[end];
    geometry_msgs::Point test_point;
    float max_deviation = 0;
    float deviation;
    for (size_t i = start; i<=end; i++)
    {
      test_point = path[i];
      deviation = pnt2line_dist(Point2Pos(start_point), Point2Pos(end_point), Point2Pos(test_point));

      if (deviation > max_deviation)
        max_deviation = deviation;
    }

    if (max_deviation > th_dist)
      return true;
    else
      return false;
  }
  //---
  void check_path_los(std::vector<bool> &los_list)
  {
    CUDA_GEO::pos p_vehicle(m_curr_ref.p.x, m_curr_ref.p.y, m_curr_ref.p.z);
    CUDA_GEO::coord c_vehicle = m_hst_map->pos2coord(p_vehicle);
    for (int i=0; i < m_guide_line.pts.size(); i++)
    {
      CUDA_GEO::pos p_path = Point2Pos(m_guide_line.pts[i]);
      CUDA_GEO::coord c_path = m_hst_map->pos2coord(p_path);
      los_list[i]=m_hst_map->isLOS(c_vehicle,c_path,0.5);
    }
  }
  //---
  int find_closest_point_on_path(const std::vector<bool> &los_list, bool require_los = true)
  {
    float min_dist=10000.0f;
    int min_id = -1;
    float dist;
    for (int i=0; i < m_guide_line.pts.size(); i++)
    {
      if (require_los && !los_list[i])
        continue;

      dist = calculate_distance(m_curr_ref.p, m_guide_line.pts[i]);
      if(dist < min_dist)
      {
        min_dist = dist;
        min_id = i;
      }
    }
    return min_id;
  }
  //---
  void extract_carrot(UAV::SingleTargetEvaluator::Target& carrot)
  {
    //Find those points that are LOS to the vehicle
    std::vector<bool> los_list(m_guide_line.pts.size());
    check_path_los(los_list);

    //Find closest LOS point on the path
    int min_id = find_closest_point_on_path(los_list);
    if(min_id < 0)
      return;

    // Find the carrot
    // It needs to be LOS, and curvature limited
    for (int i=min_id; i < m_guide_line.pts.size(); i++)
    {
      //checks LOS
      if (!los_list[i])
        break;

      //check is curvature too big
      if (is_curvature_too_big(m_guide_line.pts,min_id,i))
        break;

      //actually update the carrot
      carrot.s.p = Point2float3(m_guide_line.pts[i]);
    }

    // For viewing the carrot in RVIZ
    pcl::PointXYZ clrP;
    clrP.x = carrot.s.p.x;
    clrP.y = carrot.s.p.y;
    clrP.z = carrot.s.p.z;
    m_carrot_pnt_cld->push_back(clrP);
    m_carrot_pub.publish(m_carrot_pnt_cld);
    m_carrot_pnt_cld->clear();
  }
//---
private:
  ros::Subscriber m_guide_line_sub;
  ros::Timer m_planning_timer;

  ros::Publisher m_ref_pub;
  ros::Publisher topology_paths_pub;

  bool m_guide_line_received;

  PSO::Planner<SIMPLE_UAV> *m_pso_planner;
  PSO::Planner<EMERGENT_UAV> *m_emergent_planner;
  UAV::SingleTargetEvaluator::Target m_carrot;
  cpc_motion_planning::guide_line m_guide_line;
  UAV::UAVModel::State m_curr_ref;
  JLT::State m_curr_yaw_ref;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_plan_cycle;
  int m_ref_start_idx;
  UAVHeadingSolver m_head_sov;
  std::vector<UAV::UAVModel::State> m_traj;
  std::vector<JLT::State> m_yaw_traj;
  int m_start_braking_cycle;
  UAV::UAVRepulsiveField m_rep_filed;
  float m_take_off_height;
  bool m_planning_started;

  ros::Publisher m_carrot_pub;
  PointCloud::Ptr m_carrot_pnt_cld;
  float m_leap_height;
};

#endif // UAV_ASTAR_MOTION_PLANNER_H
