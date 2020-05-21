#include "cpc_motion_planning/uav_motion_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_srvs/Empty.h>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

UAVMotionPlanner::UAVMotionPlanner():
  m_received_map(false),
  m_pose_received(false),
  m_goal_received(false),
  m_edt_map(nullptr)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &UAVMotionPlanner::map_call_back, this);
  m_pose_sub = m_nh.subscribe("/mavros/position/local", 1, &UAVMotionPlanner::vehicle_pose_call_back, this);
  m_goal_sub = m_nh.subscribe("/move_base_simple/goal",1,&UAVMotionPlanner::goal_call_back, this);

  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_ctrl_pub = m_nh.advertise<PointCloud> ("ctrl_pnt", 1);
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UAVMotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<SIMPLE_UAV>(150,30,1);
  m_pso_planner->initialize(false);

  m_ref_gen_planner = new PSO::Planner<SIMPLE_UAV>(1,1,1);
  m_ref_gen_planner->initialize(true);
  m_traj_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_traj_pnt_cld->header.frame_id = "/world";

  m_ctrl_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_ctrl_pnt_cld->header.frame_id = "/world";


  //Initialize the control message
  m_ref_msg.rows = 12;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;

  m_yaw_target = 0;
  m_yaw_limit.vMax = 1;
  m_yaw_limit.vMin = -1;
  m_yaw_limit.aMax = 1;
  m_yaw_limit.aMin = -1;
  m_yaw_limit.jMax = 2;
  m_yaw_limit.jMin = -2;
}

UAVMotionPlanner::~UAVMotionPlanner()
{
  if (m_edt_map)
  {
    m_edt_map->free_device();
    delete m_edt_map;
  }

  m_pso_planner->release();
  delete m_pso_planner;

  m_ref_gen_planner->release();
  delete m_ref_gen_planner;
}

void UAVMotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  if (!m_pose_received || !m_received_map || !m_goal_received)
    return;

  UAV::UAVModel::State s = m_curr_ref;

  float3 diff = m_goal.s.p - s.p;
  diff.z = 0;
  double dist = sqrt(dot(diff,diff));
  if (dist > 0.5)
  {
      m_yaw_target = atan2(diff.y,diff.x);
      m_yaw_target = m_yaw_target - m_yaw_state.p;
      m_yaw_target = m_yaw_target - floor((m_yaw_target + M_PI) / (2 * M_PI)) * 2 * M_PI;
      m_yaw_target = m_yaw_target + m_yaw_state.p;
  }

  m_pso_planner->set_problem(s,m_goal);
  m_ref_gen_planner->set_problem(s,m_goal);

  auto start = std::chrono::steady_clock::now();
  JLT::TPBVPParam yaw_param;
  m_yaw_planner.solveTPBVP(m_yaw_target,0,m_yaw_state,m_yaw_limit,yaw_param);
  m_pso_planner->plan(*m_edt_map);

  auto end = std::chrono::steady_clock::now();
  std::cout << "Consumed: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost<<std::endl;

  //PSO::State cmd_state;
  std::vector<UAV::UAVModel::State> traj = m_ref_gen_planner->generate_trajectory(m_pso_planner->result.best_loc);
  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;
  float t = 0.0f;
  float u_yaw = 0;
  for (UAV::UAVModel::State traj_s : traj)
  {
    t += PSO::PSO_CTRL_DT;
    JLT::State tmp_yaw_state = m_yaw_planner.TPBVPRefGen(yaw_param,t,u_yaw);

    pcl::PointXYZ clrP;
    clrP.x = traj_s.p.x;
    clrP.y = traj_s.p.y;
    clrP.z = traj_s.p.z;
    m_traj_pnt_cld->points.push_back(clrP);


//    clrP.x = m_pso_planner->result.best_loc[i].x;
//    clrP.y = m_pso_planner->result.best_loc[i].y;
//    clrP.z = m_pso_planner->result.best_loc[i].z;
//    m_ctrl_pnt_cld->points.push_back(clrP);

    ref_counter++;
    //std::cout<<"b: "<<ref_counter<<std::endl;
    m_ref_msg.ids.push_back(ref_counter);
    m_ref_msg.data.push_back(traj_s.p.x);
    m_ref_msg.data.push_back(traj_s.p.y);
    m_ref_msg.data.push_back(traj_s.p.z);

    m_ref_msg.data.push_back(traj_s.v.x);
    m_ref_msg.data.push_back(traj_s.v.y);
    m_ref_msg.data.push_back(traj_s.v.z);

    m_ref_msg.data.push_back(traj_s.a.x);
    m_ref_msg.data.push_back(traj_s.a.y);
    m_ref_msg.data.push_back(traj_s.a.z);

    m_ref_msg.data.push_back(tmp_yaw_state.p);
    m_ref_msg.data.push_back(tmp_yaw_state.v);
    m_ref_msg.data.push_back(tmp_yaw_state.a);

    if (ref_counter == next_ref_start_idx)
    {
      m_curr_ref = traj_s;
      m_yaw_state = tmp_yaw_state;
    }

    cols++;
  }

  //std::cout<<"--------------"<<std::endl;
  m_ref_start_idx = next_ref_start_idx;
  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();

  m_traj_pub.publish(m_traj_pnt_cld);
  m_traj_pnt_cld->clear();

  m_ctrl_pub.publish(m_ctrl_pnt_cld);
  m_ctrl_pnt_cld->clear();

  m_plan_cycle++;
}

void UAVMotionPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  m_received_map = true;
  if (m_edt_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_edt_map = new EDTMap(origin,msg->width,edt_map_size);
    m_edt_map->setup_device();
  }
  else
  {
    m_edt_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_edt_map->m_grid_step = msg->width;
  }
  CUDA_MEMCPY_H2D(m_edt_map->m_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));
}

void UAVMotionPlanner::vehicle_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_pose_received = true;
  m_pose = *msg;
}

void UAVMotionPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

  m_goal.s.p.x = msg->pose.position.x;
  m_goal.s.p.y = msg->pose.position.y;
  m_goal.s.p.z = 1.5;
  m_goal.oa = m_goal_received;
  if (!m_goal_received)
  {
    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("engage", eReq, eRes);
  }
  m_goal_received = true;

  UAV::UAVModel::State s = m_curr_ref;

  float3 diff = m_goal.s.p - s.p;
  diff.z = 0;
  double dist = sqrt(dot(diff,diff));
  if (dist > 0.5)
  {
      float theta = atan2(diff.y,diff.x);
      m_pso_planner->m_dp_ctrl.m_theta = theta;
      m_ref_gen_planner->m_dp_ctrl.m_theta = theta;
  }

  //  double phi,theta,psi;

  //  tf::Quaternion q( msg->pose.orientation.x,
  //                    msg->pose.orientation.y,
  //                    msg->pose.orientation.z,
  //                    msg->pose.orientation.w);
  //  tf::Matrix3x3 m(q);
  //  m.getRPY(phi, theta, psi);

  //m_goal.theta = psi;
}
