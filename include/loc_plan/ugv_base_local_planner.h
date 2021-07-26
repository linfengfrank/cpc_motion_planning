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
#include <cpc_motion_planning/ugv/evaluator/ugv_nf1_evaluator.h>
#include <cpc_motion_planning/ugv/evaluator/ugv_hybrid_evaluator.h>
#include <cpc_motion_planning/ugv/controller/ugv_dp_control.h>
#include <cpc_motion_planning/ugv/controller/ugv_jlt_control.h>
#include <cpc_motion_planning/ugv/swarm/ugv_swarm.h>
#include <deque>
#include "tf/tf.h"
#include <cpc_motion_planning/path.h>

//#define PRED_STATE
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
  FULLY_REACHED,
  DROPOFF};
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
    float theta;
  };
#endif

public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

public:
  UGVLocalMotionPlanner();
  virtual ~UGVLocalMotionPlanner();

  void calculate_bounding_centres(const UGV::UGVModel::State &s, float2 &c_r, float2 &c_f) const
  {
    float2 uni_dir = make_float2(cosf(s.theta),sinf(s.theta));
    c_f = s.p + 0.25f*uni_dir;
    c_r = s.p - 0.25f*uni_dir;
  }

  float get_min_dist_from_host_edt(const UGV::UGVModel::State &s) const
  {
    float2 c_f,c_r;
    calculate_bounding_centres(s, c_r, c_f);
    return min(m_edt_map->getEDT(c_r),m_edt_map->getEDT(c_f));
  }

  bool check_collision_from_host_edt(const cpc_motion_planning::path_action &pa)
  {
    UGV::UGVModel::State s;
    bool collision = false;
    for (size_t i=0; i<pa.x.size(); i++)
    {
      s.p = make_float2(pa.x[i],pa.y[i]);
      s.theta = pa.theta[i];
      if (get_min_dist_from_host_edt(s) <0.41f)
      {
        collision = true;
        break;
      }
    }
    return collision;
  }

protected:
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);

#ifdef PRED_STATE
  void load_into_queue(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t);
#endif
  void update_reference_log(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t);
  UGV::UGVModel::State predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx, bool is_heading_ref);
  void add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter, const UGV::UGVModel::State &traj, const ros::Time &curr_t);

  template <typename T> int sgn(T val)
  {
      return (T(0) < val) - (val < T(0));
  }

  template<class Model, class Controller, class Evaluator, class Swarm>
  void calculate_trajectory(PSO::Planner<Model, Controller, Evaluator, Swarm> *planner, std::vector<UGV::UGVModel::State> &traj, bool use_de = false)
  {
    // conduct the motion planning
    if (use_de)
      planner->plan_de(*m_edt_map);
    else
      planner->plan(*m_edt_map);

    // generate the trajectory
    traj = planner->generate_trajectory();
  }

  void full_stop_trajectory(std::vector<UGV::UGVModel::State> &traj, UGV::UGVModel::State curr_s)
  {
    traj.clear();
    float dt = PSO::PSO_CTRL_DT;
    curr_s.v = 0;
    curr_s.w = 0;
    for (float t=0.0f; t<4; t+=dt)
    {
      traj.push_back(curr_s);
    }
  }

  bool is_pos_reached(const UGV::UGVModel::State &s, const UGV::UGVModel::State &tgt_state, float reaching_radius = 0.8f)
  {
    float2 p_diff = s.p - tgt_state.p;
    if (sqrtf(dot(p_diff,p_diff))<reaching_radius && fabsf(s.v) < 0.5f)
      return true;
    else
      return false;
  }

  bool is_heading_reached(const UGV::UGVModel::State &s, const UGV::UGVModel::State &tgt_state, float reaching_angle_diff = 0.2f)
  {
    float yaw_diff = s.theta - tgt_state.theta;
    yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
    if(fabsf(yaw_diff) < reaching_angle_diff && fabsf(s.w)<0.2f)
      return true;
    else
      return false;
  }

  float in_pi(float in)
  {
    return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
  }

  float un_in_pi(float in, float last)
  {
    return in_pi(in-last) + last;
  }

  float select_mes_ref(float mes, float ref, int& ctt, float th = 1.0f, int ctt_th = 5)
  {
    float output;
    float err = ref - mes;

    if (fabsf(err) > th)
      ctt++;
    else
      ctt = 0;

    if (ctt >= ctt_th)
    {
      ctt = 0;
      output = mes + sgn<float>(err)*0.5f;
    }
    else
    {
      output = ref;
    }
    return output;
  }

  float select_mes_ref_heading(bool &is_heading_ref, float mes, float ref, int& ctt, float th = 1.0f, int ctt_th = 5)
  {
    float output;
    float err = in_pi(ref - mes);

    if (fabsf(err) > th)
      ctt++;
    else
      ctt = 0;

    if (ctt >= ctt_th)
    {
      ctt = 0;
      output = mes + sgn<float>(err)*0.5f;
      is_heading_ref = false;
    }
    else
    {
      output = ref;
      is_heading_ref = true;
    }
    return in_pi(output);
  }

  float get_heading(const nav_msgs::Odometry &odom)
  {
    double phi,theta,psi;
    tf::Quaternion q(odom.pose.pose.orientation.x,
                     odom.pose.pose.orientation.y,
                     odom.pose.pose.orientation.z,
                     odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(phi, theta, psi);
    return psi;
  }
#ifdef SHOW_PC
  void plot_trajectory(const std::vector<UGV::UGVModel::State> &traj);
#endif

  void cycle_process_based_on_status();
  bool is_stuck(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state);
  bool is_stuck_instant(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state);
  bool is_stuck_instant_horizon(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state);
  bool is_stuck_lowpass(const UGV::UGVModel::State& s, const UGV::UGVModel::State &tgt_state);
  virtual void do_start() = 0;
  virtual void do_normal() = 0;
  virtual void do_stuck() = 0;
  virtual void do_emergent() = 0;
  virtual void do_braking() = 0;
  virtual void do_pos_reached() = 0;
  virtual void do_fully_reached() = 0;
  virtual void do_dropoff() = 0;
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
  bool m_create_host_edt;
#ifdef SHOW_PC
  ros::Publisher m_traj_pub;
  PointCloud::Ptr m_traj_pnt_cld;
#endif

#ifdef PRED_STATE
  std::deque<CmdLog> m_cmd_q;
#endif

  UGV::STATUS m_status;
  float m_stuck_pbty,m_lowpass_stuck_pbty;
  float m_lowpass_v,m_lowpass_w;
};

#endif // UGV_BASE_LOCAL_PLANNER_H
