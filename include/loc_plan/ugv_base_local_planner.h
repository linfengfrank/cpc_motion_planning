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
//#define ADD_DELAY
#define SHOW_PC

#ifdef ADD_DELAY
#include <message_filters/time_sequencer.h>
#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#endif
namespace UGV
{
// Local planner state
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

// Base class of local motion planner
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

protected:
  // To receive and process the EDT map (obstacle map)
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);

  // Wheel odometry callback
  void raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);

  // Slam odometry callback
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);

  // Store the reference into a queue for state prediction purpose
#ifdef PRED_STATE
  void load_into_queue(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t);
#endif

  // Store the reference into a queue for state prediction purpose, calls load_into_queue
  void update_reference_log(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t);

  // Predicte the state given current measurment in Odometry(odom) format
  UGV::UGVModel::State predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx, bool is_heading_ref);

  // Add the trajectory state into the reference message to be published
  void add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter, const UGV::UGVModel::State &traj, const ros::Time &curr_t);

  // Show the planned trajectory
#ifdef SHOW_PC
  void plot_trajectory(const std::vector<UGV::UGVModel::State> &traj);
#endif

  // Control the state machine of the local planner
  void cycle_process_based_on_status();

  // Pure virtual state functions
  virtual void do_start() = 0;
  virtual void do_normal() = 0;
  virtual void do_stuck() = 0;
  virtual void do_emergent() = 0;
  virtual void do_braking() = 0;
  virtual void do_pos_reached() = 0;
  virtual void do_fully_reached() = 0;
  virtual void do_dropoff() = 0;

  // Check whether the vehicle is stucked (through a probability score)
  bool is_stuck(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state);

  // Check whether the vehicle's current trajecotry satisfy the "stuck" condition
  bool is_stuck_instant(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state);

  // Check
  bool is_stuck_lowpass(const UGV::UGVModel::State& s, const UGV::UGVModel::State &tgt_state);

  // This function simulate a ref tracking process and return the simulated trajectory's min distance to obstacle
  float tracking_min_edt(const nav_msgs::Odometry &odom, const std::vector<UGV::UGVModel::State> &traj,
                                 float yaw_ctrl_gain, float w_scale, float exam_time = 1.0f);

  // Given a trajectory simulate tracking it from the current true initial state
  std::vector<UGV::UGVModel::State> simulate_tracking(const std::vector<UGV::UGVModel::State> &ref, const nav_msgs::Odometry &odom,
                                                                float yaw_ctrl_gain, float w_scale, float exam_time = 1.0f);

  // Find the vehicle's min distance to obstacle when traveling on a trajectory
  float traj_min_edt(const std::vector<UGV::UGVModel::State> &traj);

  // Calculate the center position of the bounding circles of the 2 circle model of the vehicle
  void calculate_bounding_centres(const UGV::UGVModel::State &s, float2 &c_r, float2 &c_f) const
  {
    float2 uni_dir = make_float2(cosf(s.theta),sinf(s.theta));
    c_f = s.p + 0.25f*uni_dir;
    c_r = s.p - 0.25f*uni_dir;
  }

  // Given a vehicle state, check its minimum distace to obstacles
  float get_dist_from_host_edt(const UGV::UGVModel::State &s) const
  {
    float2 c_f,c_r;
    calculate_bounding_centres(s, c_r, c_f);
    return min(m_edt_map->getEDT(c_r),m_edt_map->getEDT(c_f));
  }

  // Signum function
  template <typename T> int sgn(T val)
  {
      return (T(0) < val) - (val < T(0));
  }

  // Use the PSO planner to calculate the trajectory
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

  // Generate a full stop trajectory, stop at the current state (curr_s)
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

  // Check wether position is reached
  bool is_pos_reached(const UGV::UGVModel::State &s, const UGV::UGVModel::State &tgt_state, float reaching_radius = 0.8f)
  {
    float2 p_diff = s.p - tgt_state.p;
    if (sqrtf(dot(p_diff,p_diff))<reaching_radius && fabsf(s.v) < 0.5f)
      return true;
    else
      return false;
  }

  // Check wether yaw (heading) is reached
  bool is_heading_reached(const UGV::UGVModel::State &s, const UGV::UGVModel::State &tgt_state, float reaching_angle_diff = 0.2f)
  {
    float yaw_diff = s.theta - tgt_state.theta;
    yaw_diff = yaw_diff - floorf((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;
    if(fabsf(yaw_diff) < reaching_angle_diff && fabsf(s.w)<0.2f)
      return true;
    else
      return false;
  }

  // Wrap angle into -pi ~ pi
  float in_pi(float in)
  {
    return in - floor((in + M_PI) / (2 * M_PI)) * 2 * M_PI;
  }

  // Wrap angle from ~pi ~ pi to an accumulated value
  float un_in_pi(float in, float last)
  {
    return in_pi(in-last) + last;
  }

  // Select wether to use measurement or reference to be assigned to the current state
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

  // Select wether to use measurement or reference to be assigned to the current state (yaw)
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

  // Get the yaw angle
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
  bool m_create_host_edt; // A flag to determine whether to create a host cpy of the edt map
#ifdef SHOW_PC
  ros::Publisher m_traj_pub;
  ros::Publisher m_simulate_traj_pub;
  PointCloud::Ptr m_traj_pnt_cld;
#endif

#ifdef PRED_STATE
  std::deque<CmdLog> m_cmd_q; // A queue used to log the command value
#endif

  UGV::STATUS m_status; // Current state machine state (NORMAL, EMERGENT, etc...)

  float m_stuck_pbty,m_lowpass_stuck_pbty; // Stuck probability and low_pass_checker's stuck probability
  float m_lowpass_v,m_lowpass_w; // Low pass linear and angular velocity, used to check stuck

#ifdef ADD_DELAY
  message_filters::Subscriber<nav_msgs::Odometry> m_sub;
  message_filters::TimeSequencer<nav_msgs::Odometry> *m_seq;
#endif
};

#endif // UGV_BASE_LOCAL_PLANNER_H
