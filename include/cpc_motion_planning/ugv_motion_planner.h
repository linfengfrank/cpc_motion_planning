#ifndef UGV_MOTION_PLANNER_H
#define UGV_MOTION_PLANNER_H
#include <ros/ros.h>
#include <cpc_motion_planning/pso/pso_planner.h>
#include <cpc_aux_mapping/grid_map.h>
#include <nav_msgs/Odometry.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <cpc_motion_planning/ref_data.h>
#include <cpc_motion_planning/JLT.h>
#include <cpc_motion_planning/ugv/ugv_single_target_evaluator.h>
#include <cpc_motion_planning/ugv/ugv_dp_control.h>
#include <cpc_motion_planning/ugv/ugv_jlt_control.h>
#include <cpc_motion_planning/ugv/ugv_swarm.h>

#define SIMPLE_UGV UGV::UGVModel,UGV::UGVJLTControl,UGV::SingleTargetEvaluator,UGV::UGVSwarm<2>
class UGVMotionPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

public:
  UGVMotionPlanner();
  ~UGVMotionPlanner();

private:
  void plan_call_back(const ros::TimerEvent&);
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_map_sub;
  ros::Subscriber m_raw_odom_sub;
  ros::Subscriber m_slam_odom_sub;
  ros::Subscriber m_goal_sub;
  ros::Timer m_planning_timer;
  nav_msgs::Odometry m_raw_odo, m_slam_odo;

  ros::Publisher m_traj_pub;
  ros::Publisher m_ref_pub;

  bool m_received_map;
  bool m_raw_odo_received;
  bool m_slam_odo_received;
  bool m_goal_received;
  EDTMap *m_edt_map;
  PSO::Planner<SIMPLE_UGV> *m_pso_planner;
  PSO::Planner<SIMPLE_UGV> *m_display_planner;
  PointCloud::Ptr m_traj_pnt_cld;
  UGV::SingleTargetEvaluator::Target m_goal;
  float m_ref_v, m_ref_w;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;

};

#endif // UGV_MOTION_PLANNER_H
