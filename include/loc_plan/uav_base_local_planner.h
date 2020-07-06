#ifndef UAV_LOCAL_MOTION_PLANNER_H
#define UAV_LOCAL_MOTION_PLANNER_H

#include <ros/ros.h>
#include <cpc_motion_planning/pso/pso_planner.h>
#include <cpc_aux_mapping/grid_map.h>
#include <nav_msgs/Odometry.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cuda_geometry/cuda_nf1map.cuh>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <cpc_motion_planning/ref_data.h>
#include <cpc_motion_planning/JLT.h>
#include <cpc_motion_planning/uav/evaluator/uav_single_target_evaluator.h>
#include <cpc_motion_planning/uav/controller/uav_dp_control.h>
#include <cpc_motion_planning/uav/controller/uav_jlt_control.h>
#include <cpc_motion_planning/uav/swarm/uav_swarm.h>
#include <cpc_motion_planning/uav/uav_repulsive_field.h>

#define SHOWPC
namespace UAV
{
enum FLY_STATUS {AT_GROUND = 0,
                 TAKING_OFF,
                 IN_AIR,
                 STUCK,
                 EMERGENT,
                 BRAKING};
}

class UAVHeadingSolver
{
public:
  UAVHeadingSolver()
  {
    m_yaw_target = 0;
    m_yaw_limit.vMax = 1;
    m_yaw_limit.vMin = -1;
    m_yaw_limit.aMax = 2;
    m_yaw_limit.aMin = -2;
    m_yaw_limit.jMax = 2;
    m_yaw_limit.jMin = -2;
  }
  ~UAVHeadingSolver()
  {

  }

  void cal_yaw_target(const float3 &pnt, const UAV::UAVModel::State &s)
  {
    float3 diff = pnt - s.p;
    diff.z = 0;
    float dist = sqrtf(dot(diff,diff));
    if (dist > 0.5f)
    {
      m_yaw_target = atan2f(diff.y,diff.x);
      m_yaw_target = m_yaw_target - m_yaw_state.p;
      m_yaw_target = m_yaw_target - floorf((m_yaw_target + M_PI) / (2 * M_PI)) * 2 * M_PI;
      m_yaw_target = m_yaw_target + m_yaw_state.p;
    }
  }

  void cal_yaw_target_from_vel(float3 vel, const UAV::UAVModel::State &s)
  {
    vel.z = 0;
    float n_vel = sqrtf(dot(vel,vel));
    if (n_vel > 0.5f)
    {
      m_yaw_target = atan2f(vel.y,vel.x);
      m_yaw_target = m_yaw_target - m_yaw_state.p;
      m_yaw_target = m_yaw_target - floorf((m_yaw_target + M_PI) / (2 * M_PI)) * 2 * M_PI;
      m_yaw_target = m_yaw_target + m_yaw_state.p;
    }
  }

  std::vector<JLT::State> generate_yaw_traj()
  {
    std::vector<JLT::State> yaw_traj;
    float dt = PSO::PSO_CTRL_DT;
    float u_yaw;

    m_yaw_planner.solveTPBVP(m_yaw_target,0,m_yaw_state,m_yaw_limit,m_yaw_param);
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      yaw_traj.push_back(m_yaw_planner.TPBVPRefGen(m_yaw_param,t,u_yaw));
    }
    return yaw_traj;
  }

  void set_yaw_state(const JLT::State &yaw_state)
  {
    m_yaw_state = yaw_state;
  }

  float get_yaw()
  {
    return m_yaw_state.p;
  }

  float get_yaw_target()
  {
    return m_yaw_target;
  }
private:
  JLT m_yaw_planner;
  JLT::State m_yaw_state;
  JLT::Limit m_yaw_limit;
  float m_yaw_target;
  JLT::TPBVPParam m_yaw_param;
};

class UAVLocalMotionPlanner
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
public:
  UAVLocalMotionPlanner();
  ~UAVLocalMotionPlanner();

protected:
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void vehicle_pose_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void cycle_process_based_on_status();
  bool is_stuck(const std::vector<UAV::UAVModel::State> &traj, std::vector<JLT::State> yaw_traj, const float &best_cost);
  void add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter, const UAV::UAVModel::State &traj, const JLT::State &yaw_state);

  virtual void do_at_ground() = 0;
  virtual void do_taking_off() = 0;
  virtual void do_in_air() = 0;
  virtual void do_stuck() = 0;
  virtual void do_emergent() = 0;
  virtual void do_braking() = 0;

  template<class Model, class Controller, class Evaluator, class Swarm>
  void calculate_trajectory(PSO::Planner<Model, Controller, Evaluator, Swarm> *planner, std::vector<UAV::UAVModel::State> &traj)
  {
    // conduct the motion planning
    planner->plan(*m_edt_map);

    // generate the trajectory
    traj = planner->generate_trajectory();
  }

  void generate_static_traj(std::vector<UAV::UAVModel::State> &traj, const UAV::UAVModel::State &s)
  {
    traj.clear();
    float dt = PSO::PSO_CTRL_DT;
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      traj.push_back(s);
    }
  }

  UAV::UAVModel::State odom2state(const nav_msgs::Odometry &odom)
  {
    UAV::UAVModel::State state;
    state.p.x = odom.pose.pose.position.x;
    state.p.y = odom.pose.pose.position.y;
    state.p.z = odom.pose.pose.position.z;

    state.v.x = odom.twist.twist.linear.x;
    state.v.y = odom.twist.twist.linear.y;
    state.v.z = odom.twist.twist.linear.z;

    return state;
  }

#ifdef SHOWPC
  void plot_trajectory(const std::vector<UAV::UAVModel::State> &traj);
#endif

protected:
  ros::NodeHandle m_nh;
  ros::Subscriber m_map_sub;
  ros::Subscriber m_pose_sub;
  nav_msgs::Odometry m_pose;
  float m_stuck_pbty;
  bool m_received_map;
  bool m_pose_received;
  EDTMap *m_edt_map;
  UAV::FLY_STATUS m_fly_status;
#ifdef SHOWPC
  ros::Publisher m_traj_pub;
  PointCloud::Ptr m_traj_pnt_cld;
#endif

};

#endif // UAV_LOCAL_MOTION_PLANNER_H
