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
namespace UGV
{
enum STATUS {
  NORMAL = 0,
  STUCK,
  EMERGENT,
  BRAKING};
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
  ~UGVLocalMotionPlanner();

protected:
  void map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg);
  void raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);
  void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg);

#ifdef PRED_STATE
  void load_into_queue(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t)
  {
    for (int i=0; i<ref.cols; i++)
    {
      CmdLog tmp;
      tmp.t = curr_t + ros::Duration((i+1)*PSO::PSO_CTRL_DT);

      tmp.id = ref.ids[i];
      tmp.v = ref.data[i*ref.rows];
      tmp.w = ref.data[i*ref.rows + 1];
      //std::cout<<"id: "<<ref.ids[i]<<", "<<tmp.t<<std::endl;
      m_cmd_q.push_back(tmp);
    }
  }

  void update_reference_log(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t)
  {
    if(m_cmd_q.empty())
    {
      load_into_queue(ref, curr_t);
    }
    else
    {
      int diff = ref.ids[0] - m_cmd_q.front().id;
      while(static_cast<int>(m_cmd_q.size()) > diff && !m_cmd_q.empty())
      {
        m_cmd_q.pop_back();
      }
      load_into_queue(ref, curr_t);
    }
  }

  UGV::UGVModel::State predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx)
  {
    UGV::UGVModel::State s;
    s.p.x = odom.pose.pose.position.x;
    s.p.y = odom.pose.pose.position.y;
    s.s = 0;
    s.theta = psi;
    // Find the most related cmd

    while (m_cmd_q.size()>0)
    {
      if (m_cmd_q.front().t.toSec() <= odom.header.stamp.toSec())
        m_cmd_q.pop_front();
      else
        break;
    }

    for (const CmdLog &tmp : m_cmd_q)
    {
      if (tmp.id < ref_start_idx)
      {
        s.p.x = s.p.x + tmp.v*cos(s.theta)*PSO::PSO_CTRL_DT;
        s.p.y = s.p.y + tmp.v*sin(s.theta)*PSO::PSO_CTRL_DT;
        s.theta = s.theta + tmp.w*PSO::PSO_CTRL_DT;
      }
      else
      {
        break;
      }
    }
    return s;
  }
#endif

protected:
  ros::NodeHandle m_nh;
  ros::Subscriber m_map_sub;
  ros::Subscriber m_raw_odom_sub;
  ros::Subscriber m_slam_odom_sub;
  ros::Publisher m_traj_pub;
  nav_msgs::Odometry m_raw_odo, m_slam_odo;

  bool m_received_map;
  bool m_raw_odo_received;
  bool m_slam_odo_received;

  EDTMap *m_edt_map;
  PointCloud::Ptr m_traj_pnt_cld;

#ifdef PRED_STATE
  std::deque<CmdLog> m_cmd_q;
#endif

};

#endif // UGV_BASE_LOCAL_PLANNER_H
