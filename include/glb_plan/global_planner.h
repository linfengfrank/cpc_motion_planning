#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H
#include "glb_plan/map.h"
#include "mid_plan/utils/a_star.h"
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

class GlobalPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
public:
  GlobalPlanner();
  bool load_c_map(std::string filename);
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
    CUDA_GEO::coord c;
    CUDA_GEO::pos p;
    POINT pnt;
    for(c.x=0; c.x<m_width; c.x++)
    {
      for (c.y=0; c.y<m_height; c.y++)
      {
        p = m_a_map->coord2pos(c);
        m_c[toid(c.x,c.y)] = m_c_map.Grey(p.x,p.y);
      }
    }
  }

  void prepare_map_pcl()
  {
    CUDA_GEO::pos pnt;
    for (int x=0;x<m_width;x+=1)
    {
      for (int y=0;y<m_height;y+=1)
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
    pcl_conversions::toPCL(ros::Time::now(), m_map_pcl->header.stamp);
    m_map_vis_pub.publish (m_map_pcl);
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

public:
  std::vector<CUDA_GEO::pos> m_glb_path;
  ros::Timer m_show_map_timer;
  PointCloud::Ptr m_map_pcl;
  ros::Publisher m_map_vis_pub;
  PointCloud::Ptr m_path_pcl;
  ros::Publisher m_path_vis_pub;
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
  Astar *m_a_map;
  int *m_c;
  int *m_g;
  int *m_h;
  int *m_s;
  int *m_t;
};

#endif // GLOBAL_PLANNER_H
