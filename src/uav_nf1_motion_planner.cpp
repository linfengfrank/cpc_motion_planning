#include "cpc_motion_planning/uav_nf1_motion_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_srvs/Empty.h>

template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

UAVNF1MotionPlanner::UAVNF1MotionPlanner():
  m_received_map(false),
  m_pose_received(false),
  m_goal_received(false),
  m_edt_map(nullptr),
  m_nf1_map(nullptr)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &UAVNF1MotionPlanner::map_call_back, this);
  m_pose_sub = m_nh.subscribe("/mavros/position/local", 1, &UAVNF1MotionPlanner::vehicle_pose_call_back, this);
  m_goal_sub = m_nh.subscribe("/mid_layer/goal",1,&UAVNF1MotionPlanner::goal_call_back, this);

  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_ctrl_pub = m_nh.advertise<PointCloud> ("ctrl_pnt", 1);
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UAVNF1MotionPlanner::plan_call_back, this);

  m_guide_planner = new PSO::Planner<SIMPLE_UAV_NF1>(150,30,1);
  m_guide_planner->initialize(false);

  m_pso_planner = new PSO::Planner<SIMPLE_UAV_F>(150,30,1);
  m_pso_planner->initialize(false);

  m_ref_gen_planner = new PSO::Planner<SIMPLE_UAV_F>(1,1,1);
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
  m_yaw_limit.vMax = 2;
  m_yaw_limit.vMin = -2;
  m_yaw_limit.aMax = 4;
  m_yaw_limit.aMin = -4;
  m_yaw_limit.jMax = 4;
  m_yaw_limit.jMin = -4;
}

UAVNF1MotionPlanner::~UAVNF1MotionPlanner()
{
  if (m_edt_map)
  {
    m_edt_map->free_device();
    delete m_edt_map;
  }

  m_guide_planner->release();
  delete m_guide_planner;

  m_pso_planner->release();
  delete m_pso_planner;

  m_ref_gen_planner->release();
  delete m_ref_gen_planner;
}

void UAVNF1MotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  if (!m_pose_received || !m_received_map || !m_goal_received)
    return;

  UAV::UAVModel::State s = m_curr_ref;

  m_guide_planner->m_model.set_ini_state(s);
  m_pso_planner->m_model.set_ini_state(s);
  m_ref_gen_planner->m_model.set_ini_state(s);
  m_guide_planner->m_eva.m_curr_yaw = m_yaw_state.p;
  m_guide_planner->m_eva.m_curr_pos = s.p;
  m_pso_planner->m_eva.m_curr_yaw = m_yaw_state.p;
  m_pso_planner->m_eva.m_curr_pos = s.p;

  auto start = std::chrono::steady_clock::now();

  static int ctt = 0;
  if ((ctt++)%3==0)
    m_guide_planner->plan(*m_edt_map);

  //std::vector<UAV::UAVModel::State> traj_guide = m_ref_gen_planner->generate_trajectory(m_guide_planner->result.best_loc);

  float3 diff = m_guide_planner->result.best_loc[0] - s.p;
  diff.z = 0;
  double dist = sqrt(dot(diff,diff));
  if (dist > 0.5)
  {
    m_yaw_target = atan2(diff.y,diff.x);
    m_yaw_target = m_yaw_target - m_yaw_state.p;
    m_yaw_target = m_yaw_target - floor((m_yaw_target + M_PI) / (2 * M_PI)) * 2 * M_PI;
    m_yaw_target = m_yaw_target + m_yaw_state.p;
  }
  JLT::TPBVPParam yaw_param;
  m_yaw_planner.solveTPBVP(m_yaw_target,0,m_yaw_state,m_yaw_limit,yaw_param);


  m_pso_planner->m_eva.m_goal.s.p = m_guide_planner->result.best_loc[0];
  m_pso_planner->plan(*m_edt_map);
  auto end = std::chrono::steady_clock::now();
    std::cout << "Consumed: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms, cost: " << m_guide_planner->result.best_cost<<std::endl;



    pcl::PointXYZ clrPa;
    clrPa.x = m_guide_planner->result.best_loc[0].x;
    clrPa.y = m_guide_planner->result.best_loc[0].y;
    clrPa.z = m_guide_planner->result.best_loc[0].z;
    m_ctrl_pnt_cld->points.push_back(clrPa);







  //-----------------------------------------------------------------------------------------------------------------
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

void UAVNF1MotionPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
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

void UAVNF1MotionPlanner::vehicle_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_pose_received = true;
  m_pose = *msg;
}

void UAVNF1MotionPlanner::goal_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  if (m_nf1_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 m_nf1_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_nf1_map = new NF1Map(origin,msg->width,m_nf1_map_size);
    m_nf1_map->setup_device();
  }
  else
  {
    m_nf1_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_nf1_map->m_grid_step = msg->width;
  }
  CUDA_MEMCPY_H2D(m_nf1_map->m_nf1_map,msg->payload8.data(),static_cast<size_t>(m_nf1_map->m_byte_size));

  if (m_goal_received && m_curr_ref.p.z >= 1.8f && fabsf(m_curr_ref.v.z)<0.3f)
    m_pso_planner->m_eva.m_goal.oa = m_goal_received;

  m_guide_planner->m_eva.m_nf1_map = *m_nf1_map;

  if (!m_goal_received)
  {
    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("engage", eReq, eRes);
  }
  m_goal_received = true;

}
