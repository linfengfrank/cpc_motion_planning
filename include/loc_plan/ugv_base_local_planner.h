#ifndef UGV_BASE_LOCAL_PLANNER_H
#define UGV_BASE_LOCAL_PLANNER_H
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
#include <cpc_motion_planning/ugv/evaluator/ugv_single_target_evaluator.h>
#include <cpc_motion_planning/ugv/controller/ugv_dp_control.h>
#include <cpc_motion_planning/ugv/controller/ugv_jlt_control.h>
#include <cpc_motion_planning/ugv/swarm/ugv_swarm.h>
#include <deque>

#define PRED_STATE
#define SHOW_PC
namespace UGV
{
enum STATUS {
  START = 0,
  NORMAL,
  STUCK,
  EMERGENT,
  BRAKING,
  POS_REACHED,
  FULLY_REACHED};
}

class UGVLocalMotionPlanner
{
#ifdef PRED_STATE
public:
  struct CmdLog
  {
    ros::Time t;
    int id;
    float v;
    float w;
  };
#endif

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

public:
  UGVLocalMotionPlanner();
  virtual ~UGVLocalMotionPlanner();

protected:
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);

#ifdef PRED_STATE
  void load_into_queue(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t);
#endif
  void update_reference_log(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t);
  UGV::UGVModel::State predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx);

  void add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter, const UGV::UGVModel::State &traj);

  template<class Model, class Controller, class Evaluator, class Swarm>
  void calculate_trajectory(PSO::Planner<Model, Controller, Evaluator, Swarm> *planner, std::vector<UGV::UGVModel::State> &traj)
  {
    // conduct the motion planning
    planner->plan(*m_edt_map);

    // generate the trajectory
    traj = planner->generate_trajectory();
  }

  void full_stop_trajectory(std::vector<UGV::UGVModel::State> &traj)
  {
    traj.clear();
    float dt = PSO::PSO_CTRL_DT;;
    for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
    {
      UGV::UGVModel::State s;
      s.v = 0;
      s.w = 0;
      traj.push_back(s);
    }
  }

  bool is_pos_reached(const UGV::UGVModel::State &s, const UGV::UGVModel::State &tgt_state)
  {
    float2 p_diff = s.p - tgt_state.p;
    if (sqrtf(dot(p_diff,p_diff))<0.5)
      return true;
    else
      return false;
  }

  bool is_heading_reached(const UGV::UGVModel::State &s, const UGV::UGVModel::State &tgt_state)
  {
    float yaw_diff = s.theta - tgt_state.theta;
    yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
    if(fabsf(yaw_diff) < 0.2)
      return true;
    else
      return false;
  }

#ifdef SHOW_PC
  void plot_trajectory(const std::vector<UGV::UGVModel::State> &traj);
#endif

  void cycle_process_based_on_status();
  bool is_stuck(const std::vector<UGV::UGVModel::State> &traj, const float &best_cost);
  bool is_stuck_instant(const std::vector<UGV::UGVModel::State> &traj, const float &best_cost);
  bool is_stuck_instant_horizon(const std::vector<UGV::UGVModel::State> &traj, const float &best_cost);
  virtual void do_start() = 0;
  virtual void do_normal() = 0;
  virtual void do_stuck() = 0;
  virtual void do_emergent() = 0;
  virtual void do_braking() = 0;
  virtual void do_pos_reached() = 0;
  virtual void do_fully_reached() = 0;

protected:
  ros::NodeHandle m_nh;
  ros::Subscriber m_map_sub;
  ros::Subscriber m_raw_odom_sub;
  ros::Subscriber m_slam_odom_sub;
  nav_msgs::Odometry m_raw_odo, m_slam_odo;

  bool m_received_map;
  bool m_raw_odo_received;
  bool m_slam_odo_received;

  EDTMap *m_edt_map;
#ifdef SHOW_PC
  ros::Publisher m_traj_pub;
  PointCloud::Ptr m_traj_pnt_cld;
#endif

#ifdef PRED_STATE
  std::deque<CmdLog> m_cmd_q;
#endif

  UGV::STATUS m_status;
  float m_stuck_pbty;
};

#endif // UGV_BASE_LOCAL_PLANNER_H
