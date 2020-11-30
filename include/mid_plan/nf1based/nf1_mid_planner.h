#ifndef NF1_MID_PLANNER_H
#define NF1_MID_PLANNER_H

#include <ros/ros.h>
#include <mid_plan/utils/grid_graph.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <algorithm>
#include "cpc_motion_planning/ref_data.h"
#include <mid_plan/utils/dijkstra.h>
#include <mid_plan/utils/a_star.h>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include "cutt/cutt.h"
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cpc_motion_planning/path.h>
#include <tf/tf.h>
#include <cpc_aux_mapping/nf1_task.h>
#include <std_msgs/Int32MultiArray.h>
#define SHOWPC

class NF1MidPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

public:
  NF1MidPlanner();
  ~NF1MidPlanner();

private:
  void setup_map_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map, bool resize);
  void copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map);
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr& msg);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void glb_path_call_back(const cpc_motion_planning::path::ConstPtr &msg);
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void goal_reached_call_back(const std_msgs::Int32MultiArray::ConstPtr &msg);
  void set_goal(CUDA_GEO::pos goal);
  void plan(const ros::TimerEvent&);
  void prepare_line_map(const std::vector<float2> &path);

  void load_straight_line_mission(const std_msgs::Int32::ConstPtr &msg);
  std::vector<float2> get_local_path();
  void set_curr_act_path();

private:
  ros::NodeHandle m_nh;
  PointCloud::Ptr m_pclOut;

  ros::Subscriber m_map_sub;
  ros::Subscriber m_glb_tgt_sub;
  ros::Subscriber m_straight_line_mission_sub;
  ros::Subscriber m_slam_odom_sub;
  ros::Subscriber m_glb_path_sub;
  ros::Subscriber m_goal_reach_sub;

  ros::Publisher m_nf1_pub;
  ros::Publisher m_pc_pub;
  ros::Publisher m_mid_goal_pub;
  ros::Publisher m_straight_line_vis_pub;

  Dijkstra *m_d_map=nullptr;
  Astar *m_a_map=nullptr;
  ros::Timer m_glb_plan_timer;
  cpc_aux_mapping::nf1_task m_nf1_msg;
  bool m_received_map;
  bool m_received_goal;
  bool m_line_following_mode;
  bool m_received_odom;
  CUDA_GEO::pos m_goal;
  EDTMap* m_line_map=nullptr;
  cuttHandle m_rot_plan[2];

  std::vector<float2> m_curr_act_path;
  int m_curr_act_id;
  cpc_motion_planning::path m_path;
  float3 m_curr_pose;

  int m_closest_pnt_idx;

  // helper functions
private:
  void set_nf1_task_info(unsigned char type, int path_id, int act_id, float3 goal)
  {
    m_nf1_msg.drive_type = type;
    m_nf1_msg.act_id = act_id;
    m_nf1_msg.path_id = path_id;
    m_nf1_msg.goal_x = goal.x;
    m_nf1_msg.goal_y = goal.y;
    m_nf1_msg.goal_theta = goal.z;
  }

  void publish_mid_goal(CUDA_GEO::coord mid_goal)
  {
    geometry_msgs::PoseStamped mid_goal_pose;
    CUDA_GEO::pos mid_goal_pos = m_d_map->coord2pos(mid_goal);
    mid_goal_pose.header.frame_id="world";
    mid_goal_pose.pose.position.x = mid_goal_pos.x;
    mid_goal_pose.pose.position.y = mid_goal_pos.y;
    mid_goal_pose.pose.position.z = mid_goal_pos.z;
    m_mid_goal_pub.publish(mid_goal_pose);
  }
  void publishMap()
  {
    //publish the point cloud to rviz for checking
    CUDA_GEO::coord c;
    CUDA_GEO::pos p;
    for (int x=0;x<m_d_map->getMaxX();x++)
    {
      for (int y=0;y<m_d_map->getMaxY();y++)
      {

        c.x = x;
        c.y = y;
        c.z = 0;
        float d_c = m_d_map->getCost2Come(c,0.0f)*3;
        if (d_c > 255) d_c = 255;
        int d = static_cast<int>(d_c);
        p = m_d_map->coord2pos(c);
        pcl::PointXYZRGB clrP;
        clrP.x = p.x;
        clrP.y = p.y;
        clrP.z = p.z;

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
        m_pclOut->points.push_back (clrP);
      }
    }

    CUDA_GEO::coord shift(m_d_map->getMaxX()/2,m_d_map->getMaxY()/2,0);
    shift.x -= m_line_map->m_map_size.x/2;
    shift.y -= m_line_map->m_map_size.y/2;
    shift.z -= m_line_map->m_map_size.z/2;
    for (c.x=0;c.x<m_line_map->m_map_size.x;c.x++)
    {
      for (c.y=0;c.y<m_line_map->m_map_size.y;c.y++)
      {
        for (c.z=0;c.z<m_line_map->m_map_size.z;c.z++)
        {
          int idx = m_line_map->to_id(c.x, c.y, c.z);
          if (m_line_map->m_hst_sd_map[idx].o)
          {
            CUDA_GEO::pos p = m_d_map->coord2pos(c+shift);
            pcl::PointXYZRGB clrP;
            clrP.x = p.x;
            clrP.y = p.y;
            clrP.z = p.z;
            clrP.r = 255;
            clrP.g = 255;
            clrP.b = 255;
            m_pclOut->points.push_back (clrP);
          }
        }
      }
    }
    pcl_conversions::toPCL(ros::Time::now(), m_pclOut->header.stamp);
    m_pc_pub.publish (m_pclOut);
    m_pclOut->clear();
  }
  //---
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

    for (float2 wp: wps)
    {
      geometry_msgs::Point pnt;
      pnt.x = wp.x;
      pnt.y = wp.y;
      pnt.z = 0;
      line_strip.points.push_back(pnt);
    }
    m_straight_line_vis_pub.publish(line_strip);
  }

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

  std::vector<float2> cascade_vector(const std::vector<float2> &A, const std::vector<float2> &B)
  {
    std::vector<float2> AB;
    AB.reserve( A.size() + B.size() ); // preallocate memory
    AB.insert( AB.end(), A.begin(), A.end() );
    AB.insert( AB.end(), B.begin(), B.end() );
    return AB;
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

  bool is_curvature_too_big(const std::vector<float2> &path, size_t start, size_t end)
  {
    float2 start_point = path[start];
    float2 end_point = path[end];
    float2 test_point;
    float max_deviation = 0;
    float deviation;
    for (size_t i = start; i<=end; i++)
    {
      test_point = path[i];
      deviation = pnt2line_dist(start_point, end_point, test_point);

      if (deviation > max_deviation)
        max_deviation = deviation;
    }

    if (max_deviation > 0.15f)
      return true;
    else
      return false;
  }

  std::vector<float2> get_curvature_bounded_path(const std::vector<float2> &path)
  {
    std::vector<float2> output;
    for (size_t i = 0; i< path.size(); i++)
    {
      if(!is_curvature_too_big(path,0,i))
      {
        output.push_back(path[i]);
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
};

#endif // NF1_MID_PLANNER_H
