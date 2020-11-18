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

#define SHOWPC

class NF1MidPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

public:
  NF1MidPlanner();

private:
  void setup_map_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map, bool resize);
  void copy_map_to_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map);
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr& msg);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void set_goal(CUDA_GEO::pos goal);
  void plan(const ros::TimerEvent&);

private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_map_sub;
  ros::Subscriber m_glb_tgt_sub;

  PointCloud::Ptr m_pclOut;
  ros::Publisher m_nf1_pub;
  ros::Publisher m_pc_pub;
  ros::Publisher m_mid_goal_pub;
  Dijkstra *m_d_map=nullptr;
  Astar *m_a_map=nullptr;
  ros::Timer m_glb_plan_timer;
  cpc_aux_mapping::grid_map m_nf1_map_msg;
  bool m_received_map;
  bool m_received_goal;
  CUDA_GEO::pos m_goal;

  // helper functions
private:
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
        //for (int z=0;z<msg->z_length;z++)
        {
          c.x = x;
          c.y = y;
          c.z = 0;
          float d_c = m_d_map->getCost2Come(c,0.0f)*15;
          if (d_c > 255) d_c = 255;
          int d = static_cast<int>(d_c);

          //                std::cout<<d<<" ";
          //int color = (EDTMap::MAX_RANGE*EDTMap::MAX_RANGE-d)*255/EDTMap::MAX_RANGE/EDTMap::MAX_RANGE;
          //if (d < 4.1)
          {
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
      }
    }
    pcl_conversions::toPCL(ros::Time::now(), m_pclOut->header.stamp);
    m_pc_pub.publish (m_pclOut);
    m_pclOut->clear();
  }
};

#endif // NF1_MID_PLANNER_H
