#include "cpc_motion_planning/motion_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_srvs/Empty.h>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

MotionPlanner::MotionPlanner():
  m_received_map(false),
  m_pose_received(false),
  m_goal_received(false),
  m_edt_map(nullptr)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &MotionPlanner::map_call_back, this);
  m_pose_sub = m_nh.subscribe("/mavros/position/local", 1, &MotionPlanner::vehicle_pose_call_back, this);
  m_goal_sub = m_nh.subscribe("/move_base_simple/goal",1,&MotionPlanner::goal_call_back, this);

  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_ctrl_pub = m_nh.advertise<PointCloud> ("ctrl_pnt", 1);
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &MotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner(100,40,3);
  m_pso_planner->load_data_matrix();
  m_pso_planner->create_particles();

  m_display_planner = new PSO::Planner(1,1,1);
  m_display_planner->load_data_matrix(true);
  m_traj_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_traj_pnt_cld->header.frame_id = "/world";

  m_ctrl_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_ctrl_pnt_cld->header.frame_id = "/world";


  //Initialize the control message
  m_ref_msg.rows = 9;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;
}

MotionPlanner::~MotionPlanner()
{
  if (m_edt_map)
  {
    m_edt_map->free_device();
    delete m_edt_map;
  }

  m_pso_planner->free_data_matrix();
  m_pso_planner->delete_particles();
  delete m_pso_planner;

  m_display_planner->free_data_matrix(true);
  delete m_display_planner;
}

void MotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  if (!m_pose_received || !m_received_map || !m_goal_received)
    return;

  PSO::State s = m_curr_ref;

  auto start = std::chrono::steady_clock::now();
  m_pso_planner->plan(s,m_goal,*m_edt_map);
  auto end = std::chrono::steady_clock::now();
  std::cout << "Consumed: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost<<std::endl;

  //PSO::State cmd_state;
  float dt = PSO::PSO_CTRL_DT;
  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+2;
  for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
  {
    int i = static_cast<int>(floor(t/PSO::PSO_STEP_DT));
    if (i > PSO::PSO_STEPS - 1)
      i = PSO::PSO_STEPS - 1;

    float3 u = PSO::dp_control(s, m_pso_planner->result.best_loc[i], m_display_planner->m_carrier, m_display_planner->m_ubc);
    PSO::model_forward(s,u,dt);

    pcl::PointXYZ clrP;
    clrP.x = s.p.x;
    clrP.y = s.p.y;
    clrP.z = s.p.z;
    m_traj_pnt_cld->points.push_back(clrP);


    clrP.x = m_pso_planner->result.best_loc[i].x;
    clrP.y = m_pso_planner->result.best_loc[i].y;
    clrP.z = m_pso_planner->result.best_loc[i].z;
    m_ctrl_pnt_cld->points.push_back(clrP);

    ref_counter++;
    //std::cout<<"b: "<<ref_counter<<std::endl;
    m_ref_msg.ids.push_back(ref_counter);
    m_ref_msg.data.push_back(s.p.x);
    m_ref_msg.data.push_back(s.p.y);
    m_ref_msg.data.push_back(s.p.z);

    m_ref_msg.data.push_back(s.v.x);
    m_ref_msg.data.push_back(s.v.y);
    m_ref_msg.data.push_back(s.v.z);

    m_ref_msg.data.push_back(s.a.x);
    m_ref_msg.data.push_back(s.a.y);
    m_ref_msg.data.push_back(s.a.z);

    if (ref_counter == next_ref_start_idx)
      m_curr_ref = s;

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

void MotionPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
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

void MotionPlanner::vehicle_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_pose_received = true;
  m_pose = *msg;
}

void MotionPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
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

  //  double phi,theta,psi;

  //  tf::Quaternion q( msg->pose.orientation.x,
  //                    msg->pose.orientation.y,
  //                    msg->pose.orientation.z,
  //                    msg->pose.orientation.w);
  //  tf::Matrix3x3 m(q);
  //  m.getRPY(phi, theta, psi);

  //m_goal.theta = psi;
}
