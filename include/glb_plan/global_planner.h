#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H
#include "mid_plan/utils/a_star.h"
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <cpc_motion_planning/path.h>
#include <glb_plan/path_smoother.h>
#include <cpc_motion_planning/glb_plan_srv.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <cpc_motion_planning/exe_recorded_path.h>
#include <comm_mapping_shared_headers/map.h>
#include <ros_map/map_service.h>

class GlobalPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
public:
  GlobalPlanner();
  ~GlobalPlanner();
  void prepare_c_map();
  std::vector<CUDA_GEO::pos> plan(const CUDA_GEO::pos &goal, const CUDA_GEO::pos &start);
  bool exe_recorded_path(cpc_motion_planning::exe_recorded_path::Request &req,
                         cpc_motion_planning::exe_recorded_path::Response &res);
private:
  //-------------------------------------------
  // Euclidean Distance Transform related code|
  //-------------------------------------------
  int toid(int x, int y)
  {
    return y*m_width + x;
  }
  //---
  inline int f(int x,int i,int y)
  {
      return (x-i)*(x-i) + m_g[toid(i,y)]*m_g[toid(i,y)];
  }
  //---
  inline int sep(int i,int u,int y)
  {
      return (u*u-i*i+ m_g[toid(u,y)]*m_g[toid(u,y)] - m_g[toid(i,y)]*m_g[toid(i,y)])/(2*(u-i));
  }
  //---
  void perform_edt()
  {
    for (int x=0;x<m_width; x++)
    {
      phase1(x);
    }

    for (int y=0;y<m_height; y++)
    {
      phase2(y);
    }

    for (int x=0; x<m_width; x++)
    {
      for (int y=0; y<m_height; y++)
      {
        m_a_map->getEdtMapPtr()[toid(x,y)].d = sqrtf(m_h[toid(x,y)]);
      }
    }
  }
  //---
  void phase1(int x)
  {
    // scan 1
    if (m_c[toid(x,0)] > 100)
      m_g[toid(x,0)]=0;
    else
      m_g[toid(x,0)] = 10000;

    for (int y=1; y<m_height; y++)
    {
      if (m_c[toid(x,y)] > 100)
        m_g[toid(x,y)]=0;
      else
        m_g[toid(x,y)] = 1 + m_g[toid(x,y-1)];
    }
    // scan 2
    for (int y=m_height-2; y>=0; y--)
    {
      if (m_g[toid(x,y+1)] < m_g[toid(x,y)])
      {
        m_g[toid(x,y)] = 1+m_g[toid(x,y+1)];
      }
    }
  }
  //---
  void phase2(int y)
  {
    int q=0;
    m_s[toid(0,y)] = 0;
    m_t[toid(0,y)] = 0;
    for (int u=1;u<m_width;u++)
    {
      while (q>=0 && f(m_t[toid(q,y)],m_s[toid(q,y)],y)
             > f(m_t[toid(q,y)],u,y))
      {
        q--;
      }
      if (q<0)
      {
        q = 0;
        m_s[toid(0,y)]=u;
      }
      else
      {
        int w = 1+sep(m_s[toid(q,y)],u,y);
        if (w < m_width)
        {
          q++;
          m_s[toid(q,y)]=u;
          m_t[toid(q,y)]=w;
        }
      }
    }
    for (int u=m_width-1;u>=0;u--)
    {
      m_h[toid(u,y)]=f(u,m_s[toid(q,y)],y);
      if (u == m_t[toid(q,y)])
        q--;
    }
  }

  //--------------------------------------
  // Global map loading related functions|
  //--------------------------------------
  void build_axis_aligned_map()
  {
    m_c_map.crop(m_origin.x,m_origin.y,m_width,m_height,m_step_width,m_c);
  }
  //---
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
  //---
  bool read_c_map(int map_idx)
  {
    ros_map::map_service srv;
    srv.request.type = ros_map::map_service::Request::REQUEST_MAP_INFO;
    srv.request.param = map_idx;
    if (m_cmap_client.call(srv))
    {
      MAP_INFO map_info;
      string_to_map_info(srv.response.response.c_str(), &map_info);
      m_c_map.load(&map_info);
      return true;
    }
    else
    {
      return false;
    }
  }

  //-----------------------------------
  // Global planning related functions|
  //-----------------------------------
  bool glb_plan_service(cpc_motion_planning::glb_plan_srv::Request &req,
                        cpc_motion_planning::glb_plan_srv::Response &res);
  void exe_curr_glb_plan(const std_msgs::Bool::ConstPtr &msg);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void set_goal(CUDA_GEO::pos goal);
  void go_home(const std_msgs::Bool::ConstPtr &msg);
  //---
  void prepare_glb_path()
  {
    cpc_motion_planning::path glb_path;
    glb_path.request_ctt = m_glb_path_id++;

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
      m_glb_path_msg = glb_path;
    }
  }
  //---
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
  //---
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
  //---
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
  //---
  float in_pi(float in)
  {
    return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
  }
  //---
  std::vector<float2> make_straight_path(const float2 &a, const float2 &b)
  {
    std::vector<float2> path;
    float2 delta = b-a;
    float dist = sqrtf(dot(delta,delta));

    if (dist < 0.1f)
    {
      path.push_back(b);
      return path;
    }

    delta = delta/dist;
    for (float l = 0; l <= dist; l+=0.1f)
    {
      path.push_back(a+delta*l);
    }
    return path;
  }
  //---
  std::vector<size_t> split_merge(const std::vector<float2> &path)
  {
    std::vector<size_t> split_idx;

    // If there is no path, return the center point
    if (path.size() <= 1)
    {
      return split_idx;
    }

    std::stack<std::pair<unsigned int,unsigned int>> task; // fist is anchor, second is target
    task.push(std::make_pair(path.size() - 1, 0));

    unsigned int check = static_cast<unsigned int>(path.size()) - 1;

    split_idx.push_back(path.size() - 1);
    while (task.size() > 0)
    {
      unsigned int target = task.top().second;
      unsigned int anchor = task.top().first;
      task.pop();
      // find the largest distance
      unsigned int max_id = 0;
      float max_dist = 0;
      for (unsigned int j = target; j< anchor; j++)
      {
        float dist = pnt2line_dist(path[anchor],path[target],path[j]);
        if (dist > max_dist)
        {
          max_dist = dist;
          max_id = j;
        }
      }

      if (max_dist > 0.2f)
      {
        task.push(std::make_pair(max_id, target));
        task.push(std::make_pair(anchor, max_id));
        target = max_id;
      }
      else
      {
        split_idx.push_back(target);
        if (target >= check)
        {
          std::cout<<"path size: "<<path.size()<<std::endl;
          ROS_ERROR("Wrong split sequence");
          exit(-1);
        }
        check = target;
      }
    }
    std::reverse(split_idx.begin(),split_idx.end());
    return split_idx;
  }
  //---
  std::vector<float2> cascade_vector(const std::vector<float2> &A, const std::vector<float2> &B)
  {
    std::vector<float2> AB;
    AB.reserve( A.size() + B.size() ); // preallocate memory
    AB.insert( AB.end(), A.begin(), A.end() );
    AB.insert( AB.end(), B.begin(), B.end() );
    return AB;
  }
  //---
  void publish_glb_path(const cpc_motion_planning::path &path)
  {
    if (!m_auto_mission_started)
    {
      m_auto_mission_started = true;
      m_home_position.x = m_curr_pose.x;
      m_home_position.y = m_curr_pose.y;
    }

    m_glb_path_pub.publish(path);
  }

  //------------------------
  // Visulization functions|
  //------------------------
  void show_glb_map(const ros::TimerEvent&)
  {
    if (m_map_vis_pub.getNumSubscribers() > 0)
    {
      pcl_conversions::toPCL(ros::Time::now(), m_map_pcl->header.stamp);
      m_map_vis_pub.publish (m_map_pcl);
      m_show_map_timer.stop();
    }
  }
  //---
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

private:
  int m_glb_path_id;
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
  ros::Subscriber m_go_home_sub;
  ros::ServiceServer m_recorded_path_srv;
  float3 m_curr_pose;
  CUDA_GEO::pos m_goal;
  CUDA_GEO::pos m_origin;
  float m_step_width;
  int m_width;
  int m_height;
  std::string m_cmap_filename;
  Map m_c_map;
  PathSmoother *m_ps;
  Astar *m_a_map;
  unsigned char *m_c;
  int *m_g;
  int *m_h;
  int *m_s;
  int *m_t;
  ros::ServiceServer m_glb_plan_srv;
  ros::Subscriber m_glb_plan_execute_sub;
  std::vector<float2> m_curr_act_path;
  cpc_motion_planning::path m_glb_path_msg;
  float m_safety_radius;
  float2 m_home_position;
  bool m_auto_mission_started;
  ros::ServiceClient m_cmap_client;
};

#endif // GLOBAL_PLANNER_H
