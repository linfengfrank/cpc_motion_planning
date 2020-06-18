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
#include <cpc_motion_planning/uav/uav_single_target_evluator.h>
#include <cpc_motion_planning/uav/uav_dp_control.h>
#include <cpc_motion_planning/uav/uav_jlt_control.h>
#include <cpc_motion_planning/uav/uav_swarm.h>

namespace UAV
{
enum FLY_STATUS {AT_GROUND = 0,
                 TAKING_OFF,
                 IN_AIR,
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

  void cal_from_pnt(const float3 &pnt, const UAV::UAVModel::State &s)
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
    m_yaw_planner.solveTPBVP(m_yaw_target,0,m_yaw_state,m_yaw_limit,m_yaw_param);
  }

  std::vector<JLT::State> generate_yaw_traj()
  {
    std::vector<JLT::State> yaw_traj;
    float dt = PSO::PSO_CTRL_DT;
    float u_yaw;
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
private:
  JLT m_yaw_planner;
  JLT::State m_yaw_state;
  JLT::Limit m_yaw_limit;
  float m_yaw_target;
  JLT::TPBVPParam m_yaw_param;
};

class UAVLocalMotionPlanner
{
public:
  UAVLocalMotionPlanner();
  ~UAVLocalMotionPlanner();

protected:
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void vehicle_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void run_state();
  bool is_stuck(const std::vector<UAV::UAVModel::State> &traj, std::vector<JLT::State> yaw_traj, const float &best_cost);
  virtual void do_at_ground() = 0;
  virtual void do_taking_off() = 0;
  virtual void do_in_air() = 0;

  template<class Model, class Controller, class Evaluator, class Swarm>
  void calculate_trajectory(PSO::Planner<Model, Controller, Evaluator, Swarm> *planner, std::vector<UAV::UAVModel::State> &traj,
                             const UAV::UAVModel::State &s, const float &yaw)
  {
    // set the current initial state
    planner->m_model.set_ini_state(s);
    planner->m_eva.m_curr_yaw = yaw;
    planner->m_eva.m_curr_pos = s.p;

    // conduct the motion planning
    planner->plan(*m_edt_map);

    // generate the trajectory
    traj = planner->generate_trajectory();
  }

protected:
  ros::NodeHandle m_nh;
  ros::Subscriber m_map_sub;
  ros::Subscriber m_pose_sub;

  geometry_msgs::PoseStamped m_pose;
  float m_stuck_pbty;


  bool m_received_map;
  bool m_pose_received;

  EDTMap *m_edt_map;
  UAV::FLY_STATUS m_fly_status;

};

#endif // UAV_LOCAL_MOTION_PLANNER_H
