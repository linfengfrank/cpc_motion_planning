#ifndef STATE_MACHINE_BLACK_BOARD_H
#define STATE_MACHINE_BLACK_BOARD_H

#include <ros/ros.h>
#include <cuda_geometry/cuda_edtmap.cuh>
#include <cpc_aux_mapping/nf1_task.h>
#include <cuda_geometry/cuda_nf1_desired_theta.cuh>
#include <cpc_motion_planning/ugv/evaluator/ugv_nf1_evaluator.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <cpc_motion_planning/ref_data.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define SHOW_PC
//#define ADD_DELAY

#ifdef ADD_DELAY
#include <message_filters/time_sequencer.h>
#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#endif

struct StampedUGVState
{
  UGV::UGVModel::State s;
  int id;
  ros::Time t;
};

class LocalBlackboard
{
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  LocalBlackboard();
  void publish_status_info(std::string &str);

  void plot_ref_trajectory(const std::vector<UGV::UGVModel::State> &traj);

  void plot_sim_trajectory(const std::vector<UGV::UGVModel::State> &traj);

  void publish_reference(const std::vector<StampedUGVState>& stamped_traj);

  // Things written on the balckboard:
public:
  bool m_goal_received = false;
  bool m_map_received = false;
  bool m_raw_odo_received = false;
  bool m_slam_odo_received = false;
  bool m_task_is_new = false;
  bool m_create_host_edt = true;
  bool m_use_adrc = true;
  uint8_t m_task_drive_dir;
  uint8_t m_exec_drive_dir = 0;
  UGV::NF1Evaluator::Target m_goal;
  UGV::UGVModel::State m_carrot;
  EDTMap *m_edt_map = nullptr;
  NF1MapDT *m_nf1_map = nullptr;
  nav_msgs::Odometry m_raw_odo, m_slam_odo;
  UGV::UGVModel::State m_init_state;
  double m_turning_efficiency = 1.0;
  float m_footprint_offset = 0.25f;
  std::vector<UGV::UGVModel::State> m_ref_traj;
  std::vector<StampedUGVState> m_stamped_ref_traj;
  cpc_motion_planning::ref_data m_ref_msg;

private:
  void nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg);

  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);

  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);

  void add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter,
                      const UGV::UGVModel::State &traj_s, const ros::Time &t);

#ifdef ADD_DELAY
  message_filters::Subscriber<nav_msgs::Odometry> m_sub;
  message_filters::TimeSequencer<nav_msgs::Odometry> *m_seq;
#endif

private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_map_sub;
  ros::Subscriber m_raw_odom_sub;
  ros::Subscriber m_slam_odom_sub;
  ros::Subscriber m_nf1_sub;
  ros::Publisher m_status_pub;
  ros::Publisher m_ref_pub;
  ros::Publisher m_traj_pub;
  ros::Publisher m_simulate_traj_pub;
  ros::Publisher m_drive_dir_pub;
  PointCloud::Ptr m_traj_pnt_cld;


};

#endif // STATE_MACHINE_BLACK_BOARD_H
