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

class MotionPlanner
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

public:
  MotionPlanner();
  ~MotionPlanner();

private:
  void plan_call_back(const ros::TimerEvent&);
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_map_sub;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_goal_sub;
    ros::Timer m_planning_timer;
    nav_msgs::Odometry m_odo;

    ros::Publisher m_traj_pub;

    bool m_received_map;
    bool m_received_state;
    bool m_goal_received;
    EDTMap *m_edt_map;
    PSO::Planner<5> *m_pso_planner;
    PSO::Planner<5> *m_display_planner;
    PointCloud::Ptr m_traj_pnt_cld;
    PSO::State m_goal;

};

#endif // MOTION_PLANNER_H
