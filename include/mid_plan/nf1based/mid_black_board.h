#ifndef MID_BLACK_BOARD_H
#define MID_BLACK_BOARD_H

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
#include <unordered_map>
#include "mid_plan/nf1based/mission_manager.h"

class MidBlackboard
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
public:
  MidBlackboard();
  ~MidBlackboard();

public:
  MissionManager m_mission_mgr;

private:
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr& msg);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void glb_path_call_back(const cpc_motion_planning::path::ConstPtr &msg);
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void goal_reached_call_back(const std_msgs::Int32MultiArray::ConstPtr &msg);
  void drive_dir_call_back(const std_msgs::Int32::ConstPtr &msg);
  void setup_map_msg(cpc_aux_mapping::grid_map &msg, GridGraph* map, bool resize);

private:
  void update_nf1_guidance_map_info(unsigned char type, int path_id, int act_id, float3 goal)
  {
    m_nf1_msg.drive_type = type;
    m_nf1_msg.act_id = act_id;
    m_nf1_msg.path_id = path_id;
    m_nf1_msg.goal_x = goal.x;
    m_nf1_msg.goal_y = goal.y;
    m_nf1_msg.goal_theta = goal.z;
  }

public:
  ros::NodeHandle m_nh;
  PointCloud::Ptr m_pclOut;

  // Subscriber, publishers and timers
  ros::Subscriber m_map_sub;
  ros::Subscriber m_glb_tgt_sub;
  ros::Subscriber m_straight_line_mission_sub;
  ros::Subscriber m_slam_odom_sub;
  ros::Subscriber m_glb_path_sub;
  ros::Subscriber m_goal_reach_sub;
  ros::Subscriber m_drive_dir_sub;
  ros::Publisher m_nf1_pub;
  ros::Publisher m_pc_pub;
  ros::Publisher m_mid_goal_pub;
  ros::Publisher m_straight_line_vis_pub;
  ros::Timer m_glb_plan_timer;

  // Map
  Dijkstra *m_d_map=nullptr;
  Astar *m_a_map=nullptr;
  EDTMap* m_line_map=nullptr;
  cuttHandle m_rot_plan[2];
  cpc_aux_mapping::nf1_task m_nf1_msg;

  // Auxilary
  float3 m_curr_pose;

  // Flags
  bool m_received_map;
  bool m_received_odom;

  // Parameters
  int m_closest_pnt_idx;
  int m_drive_dir;
  int m_clp_idx_search_start;
  int m_clp_idx_search_end;
  int m_look_ahead;
  int m_look_back;
  float m_curvature_split;
  float m_safety_radius;
  float m_mid_goal_orient;
  float m_min_carrot_dist;
  float m_max_carrot_dist;
};

#endif // MID_BLACK_BOARD_H
