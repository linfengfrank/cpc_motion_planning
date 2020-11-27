#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H
#include "edt/map.h"
#include "mid_plan/utils/a_star.h"
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <cpc_motion_planning/path.h>
#include <glb_plan/path_smoother.h>
class GlobalPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
public:
  GlobalPlanner();
  ~GlobalPlanner();
  bool load_c_map();
  void perform_edt();
  std::vector<CUDA_GEO::pos> plan(const CUDA_GEO::pos &goal, const CUDA_GEO::pos &start);

public:
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void set_goal(CUDA_GEO::pos goal);

  void phase1(int x);
  void phase2(int y);
  int toid(int x, int y)
  {
    return y*m_width + x;
  }

  inline int f(int x,int i,int y)
  {
      return (x-i)*(x-i) + m_g[toid(i,y)]*m_g[toid(i,y)];
  }

  inline int sep(int i,int u,int y)
  {
      return (u*u-i*i+ m_g[toid(u,y)]*m_g[toid(u,y)] - m_g[toid(i,y)]*m_g[toid(i,y)])/(2*(u-i));
  }

  void build_axis_aligned_map()
  {
    m_c_map.Crop(m_origin,m_width,m_height,m_step_width,m_c);
  }

  void prepare_map_pcl()
  {
    CUDA_GEO::pos pnt;
    for (int x=0;x<m_width;x+=4)
    {
      for (int y=0;y<m_height;y+=4)
      {
        int idx = toid(x,y);
        int d_c = m_h[idx];
        if (d_c > 255) d_c = 255;
        int d = 255 - static_cast<int>(d_c);
        CUDA_GEO::pos pnt = m_a_map->coord2pos(CUDA_GEO::coord(x,y,0));
        pcl::PointXYZRGB clrP;
        clrP.x = pnt.x;
        clrP.y = pnt.y;
        clrP.z = 0;

        if ( d < 128)
        {
          clrP.b = 255-2*static_cast<unsigned char>(d);
          clrP.g = 2*static_cast<unsigned char>(d);
        }
        else
        {
          clrP.g = 255 - 2*(static_cast<unsigned char>(d) - 128);
          clrP.r = 2*(static_cast<unsigned char>(d)-128);
        }
        clrP.a = 255;
        m_map_pcl->points.push_back (clrP);
      }
    }
  }

  void show_glb_map(const ros::TimerEvent&)
  {
    if (m_map_vis_pub.getNumSubscribers() > 0)
    {
      pcl_conversions::toPCL(ros::Time::now(), m_map_pcl->header.stamp);
      m_map_vis_pub.publish (m_map_pcl);
      m_show_map_timer.stop();
    }
  }

  void show_glb_path()
  {
    for (size_t i=0; i<m_glb_path.size();i++)
    {
      pcl::PointXYZRGB clrP;
      clrP.x = m_glb_path[i].x;
      clrP.y = m_glb_path[i].y;
      clrP.z = 0;
      clrP.r = 255;
      clrP.g = 255;
      clrP.b = 255;
      clrP.a = 255;
      m_path_pcl->points.push_back (clrP);
    }
    pcl_conversions::toPCL(ros::Time::now(), m_path_pcl->header.stamp);
    m_path_vis_pub.publish (m_path_pcl);
    m_path_pcl->clear();
  }

  void publish_glb_path()
  {
    cpc_motion_planning::path glb_path;

    //--- set the path action
    cpc_motion_planning::path_action pa;
    for (size_t i=0; i<m_glb_path.size();i++)
    {
      pa.x.push_back(m_glb_path[i].x);
      pa.y.push_back(m_glb_path[i].y);
      pa.theta.push_back(0);
    }
    //--- set the drive direction---
    std::vector<float2> bc_path = get_curvature_bounded_path(m_glb_path);
    if (bc_path.size() > 0 && m_odom_received)
    {
      float2 diff = bc_path.back() - make_float2(m_curr_pose.x, m_curr_pose.y);
      float angle_diff = fabsf(in_pi(atan2f(diff.y, diff.x) - m_curr_pose.z));

      if (angle_diff < M_PI/2.0)
        pa.type = cpc_motion_planning::path_action::TYPE_FORWARD;
      else
        pa.type = cpc_motion_planning::path_action::TYPE_BACKWARD;

      // publish the global map only when the drive direction can be set
      glb_path.actions.push_back(pa);
      m_glb_path_pub.publish(glb_path);
    }
  }

  inline float pnt2line_dist(const float2 & c1, const float2 & c2, const float2 & c0)
  {
    float2 a = c1-c0;
    float2 b = c2-c1;

    float a_square = dot(a,a);
    float b_square = dot(b,b);
    float a_dot_b = a.x*b.x + a.y*b.y;

    if (b_square < 1e-3)
      return sqrtf(static_cast<float>(a_square));

    return sqrtf(static_cast<float>(a_square*b_square - a_dot_b*a_dot_b)/static_cast<float>(b_square));
  }

  bool is_curvature_too_big(const std::vector<CUDA_GEO::pos> &path, size_t start, size_t end)
  {
    float2 start_point = make_float2(path[start].x, path[start].y);
    float2 end_point = make_float2(path[end].x, path[end].y);
    float2 test_point;
    float max_deviation = 0;
    float deviation;
    for (size_t i = start; i<=end; i++)
    {
      test_point = make_float2(path[i].x, path[i].y);
      deviation = pnt2line_dist(start_point, end_point, test_point);

      if (deviation > max_deviation)
        max_deviation = deviation;
    }

    if (max_deviation > 0.15f)
      return true;
    else
      return false;
  }

  std::vector<float2> get_curvature_bounded_path(const std::vector<CUDA_GEO::pos> &path)
  {
    std::vector<float2> output;
    for (size_t i = 0; i< path.size(); i++)
    {
      if(!is_curvature_too_big(path,0,i))
      {
        output.push_back(make_float2(path[i].x,path[i].y));
      }
      else
      {
        break;
      }
    }
    return output;
  }

  float in_pi(float in)
  {
    return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
  }
public:
  std::vector<CUDA_GEO::pos> m_glb_path;
  ros::Timer m_show_map_timer;
  PointCloud::Ptr m_map_pcl;
  ros::Publisher m_map_vis_pub;
  PointCloud::Ptr m_path_pcl;
  ros::Publisher m_path_vis_pub;
  ros::Publisher m_glb_path_pub;
  bool m_map_loaded;
  bool m_odom_received;
  ros::NodeHandle m_nh;
  ros::Subscriber m_slam_odom_sub;
  ros::Subscriber m_glb_tgt_sub;
  float3 m_curr_pose;
  CUDA_GEO::pos m_goal;
  CUDA_GEO::pos m_origin;
  float m_step_width;
  int m_width;
  int m_height;
  CMap m_c_map;
  PathSmoother *m_ps;
  Astar *m_a_map;
  unsigned char *m_c;
  int *m_g;
  int *m_h;
  int *m_s;
  int *m_t;
};

#endif // GLOBAL_PLANNER_H
