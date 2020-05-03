#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H
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

class MotionPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

public:
  MotionPlanner();
  ~MotionPlanner();

private:
  void plan_call_back(const ros::TimerEvent&);
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void vehicle_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_map_sub;
    ros::Subscriber m_pose_sub;
    ros::Subscriber m_goal_sub;
    ros::Timer m_planning_timer;
    geometry_msgs::PoseStamped m_pose;

    ros::Publisher m_traj_pub;
    ros::Publisher m_ctrl_pub;
    ros::Publisher m_ref_pub;

    bool m_received_map;
    bool m_pose_received;
    bool m_goal_received;
    EDTMap *m_edt_map;
    PSO::Planner *m_pso_planner;
    PSO::Planner *m_display_planner;
    PointCloud::Ptr m_traj_pnt_cld, m_ctrl_pnt_cld;
    PSO::Target m_goal;
    PSO::State m_curr_ref;
    cpc_motion_planning::ref_data m_ref_msg;
    int m_plan_cycle;
    int m_ref_start_idx;
};

#endif // MOTION_PLANNER_H
