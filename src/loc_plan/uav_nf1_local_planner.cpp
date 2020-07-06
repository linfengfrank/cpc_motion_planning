#include "loc_plan/uav_nf1_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_srvs/Empty.h>

template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

UAVNF1MotionPlanner::UAVNF1MotionPlanner():
  UAVLocalMotionPlanner(),
  m_goal_received(false),
  m_nf1_map(nullptr)
{
  m_goal_sub = m_nh.subscribe("/mid_layer/goal",1,&UAVNF1MotionPlanner::goal_call_back, this);


  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UAVNF1MotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<SIMPLE_UAV_NF1>(150,30,1);
  m_pso_planner->initialize();

  m_emergent_planner = new PSO::Planner<EMERGENT_UAV_NF1>(50,20,1);
  m_emergent_planner->initialize();
  m_emergent_planner->m_ctrl_dev.set_limit(make_float2(5,2), make_float2(4,2), make_float2(5,5));
  m_emergent_planner->m_ctrl_host.set_limit(make_float2(5,2), make_float2(4,2), make_float2(5,5));


  //Initialize the control message
  m_ref_msg.rows = 12;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;
}

UAVNF1MotionPlanner::~UAVNF1MotionPlanner()
{
  m_pso_planner->release();
  delete m_pso_planner;

  m_emergent_planner->release();
  delete m_emergent_planner;
}

void UAVNF1MotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  cycle_init();
  cycle_process_based_on_status();

  if (m_fly_status <= UAV::AT_GROUND)
    return;

  // trajectory generation
  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;
  float t = 0.0f;
  int i=0;
  for (UAV::UAVModel::State traj_s : m_traj)
  {
    t += PSO::PSO_CTRL_DT;
    JLT::State yaw_state = m_yaw_traj[i++];

    ref_counter++;
    add_to_ref_msg(m_ref_msg,ref_counter,traj_s,yaw_state);

    if (ref_counter == next_ref_start_idx)
    {
      m_curr_ref = traj_s;
      m_head_sov.set_yaw_state(yaw_state);
    }
    cols++;
  }

  m_ref_start_idx = next_ref_start_idx;
  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();

  m_plan_cycle++;
#ifdef SHOWPC
  plot_trajectory(m_traj);
#endif
}

void UAVNF1MotionPlanner::goal_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  m_goal_received = true;
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

  m_pso_planner->m_eva.m_nf1_map = *m_nf1_map;
  m_emergent_planner->m_eva.m_nf1_map = *m_nf1_map;
}



void UAVNF1MotionPlanner::do_at_ground()
{
  if (m_pose_received && m_received_map && m_goal_received)
  {
    std_srvs::Empty::Request eReq;
    std_srvs::Empty::Response eRes;
    ros::service::call("engage", eReq, eRes);
    m_fly_status = UAV::TAKING_OFF;
  }
}
void UAVNF1MotionPlanner::do_taking_off()
{
  auto start = std::chrono::steady_clock::now();
  calculate_trajectory<SIMPLE_UAV_NF1>(m_pso_planner,m_traj);
  if (m_curr_ref.p.z >= 1.8f && fabsf(m_curr_ref.v.z)<0.3f)
  {
    m_pso_planner->m_eva.m_oa = m_goal_received;
    m_pso_planner->m_eva.m_fov = m_goal_received;

    m_emergent_planner->m_eva.m_oa = m_goal_received;
    m_emergent_planner->m_eva.m_fov = m_goal_received;
    m_fly_status = UAV::IN_AIR;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;
}

void UAVNF1MotionPlanner::do_in_air()
{
  auto start = std::chrono::steady_clock::now();
  calculate_trajectory<SIMPLE_UAV_NF1>(m_pso_planner,m_traj);
  if (m_pso_planner->result.collision)
  {
    m_fly_status = UAV::EMERGENT;
    cycle_process_based_on_status();
  }
  else
  {
    m_head_sov.cal_yaw_target(m_pso_planner->result.best_loc[0], m_curr_ref);
    m_yaw_traj = m_head_sov.generate_yaw_traj();
    if(is_stuck(m_traj, m_yaw_traj, m_pso_planner->result.best_cost))
    {
      m_fly_status = UAV::STUCK;
      cycle_process_based_on_status();
    }
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;
}

void UAVNF1MotionPlanner::do_emergent()
{
  auto start = std::chrono::steady_clock::now();
  calculate_trajectory<EMERGENT_UAV_NF1>(m_emergent_planner,m_traj);
  if(m_emergent_planner->result.collision)
  {
    m_fly_status = UAV::BRAKING;
    m_start_braking_cycle = m_plan_cycle;
    m_curr_ref = odom2state(m_pose);
    cycle_process_based_on_status();
  }
  else
  {
    m_fly_status = UAV::IN_AIR;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (emergent): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_emergent_planner->result.best_cost
            << ", collision: " << m_emergent_planner->result.collision<<std::endl;
}

void UAVNF1MotionPlanner::do_braking()
{
  auto start = std::chrono::steady_clock::now();
  m_rep_filed.generate_repulse_traj(m_traj, *m_edt_map, m_curr_ref);
  if (m_plan_cycle - m_start_braking_cycle > 20)
  {
    m_fly_status = UAV::IN_AIR;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (braking): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, cost: " << m_emergent_planner->result.best_cost
            << ", collision: " << m_emergent_planner->result.collision<<std::endl;

}

void UAVNF1MotionPlanner::do_stuck()
{
  std::cout<<"stuck"<<std::endl;
  bool old_redord = m_pso_planner->m_eva.m_fov;
  m_pso_planner->m_eva.m_fov = false;
  m_pso_planner->plan(*m_edt_map);
  m_pso_planner->m_eva.m_fov = old_redord;

  std::cout<<"guiding target: "<<m_pso_planner->result.best_loc[0].x<<" "<<m_pso_planner->result.best_loc[0].y<<std::endl;
  std::cout<<"yaw target: "<<m_head_sov.get_yaw_target()<<std::endl;
  m_head_sov.cal_yaw_target(m_pso_planner->result.best_loc[0], m_curr_ref);
  m_yaw_traj = m_head_sov.generate_yaw_traj();
  m_fly_status = UAV::IN_AIR;
}

void UAVNF1MotionPlanner::cycle_init()
{
  // Set the initial states for all planners
  m_pso_planner->m_model.set_ini_state(m_curr_ref);
  m_pso_planner->m_eva.m_curr_yaw = m_head_sov.get_yaw();
  m_pso_planner->m_eva.m_curr_pos = m_curr_ref.p;

  m_emergent_planner->m_model.set_ini_state(m_curr_ref);
  m_emergent_planner->m_eva.m_curr_yaw = m_head_sov.get_yaw();
  m_emergent_planner->m_eva.m_curr_pos = m_curr_ref.p;

  // Construct default trajectories for pos and yaw
  switch (m_fly_status) {
  case UAV::AT_GROUND:
  {
    generate_static_traj(m_traj, m_curr_ref);
    break;
  }
  case UAV::TAKING_OFF:
  case UAV::IN_AIR:
  {
     m_traj = m_pso_planner->generate_trajectory();
    break;
  }
  case UAV::EMERGENT:
  {
    m_traj = m_emergent_planner->generate_trajectory();
    break;
  }
  case UAV::BRAKING:
  {
    generate_static_traj(m_traj, m_curr_ref);
    break;
  }
  default:
    break;
  }

  m_yaw_traj = m_head_sov.generate_yaw_traj();
}



