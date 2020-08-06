#include "loc_plan/ugv_sigtgt_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

UGVSigTgtMotionPlanner::UGVSigTgtMotionPlanner():
  m_goal_received(false),
  m_mid_goal_received(false),
  cycle_initialized(false),
  m_braking_start_cycle(0),
  m_nf1_map(nullptr)
{
  m_goal_sub = m_nh.subscribe("/move_base_simple/goal",1,&UGVSigTgtMotionPlanner::goal_call_back, this);
  m_mid_goal_sub = m_nh.subscribe("/mid_goal",1,&UGVSigTgtMotionPlanner::mid_goal_call_back, this);
  m_nf1_sub = m_nh.subscribe("/mid_layer/goal",1,&UGVSigTgtMotionPlanner::nf1_call_back, this);

  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_status_pub = m_nh.advertise<std_msgs::String>("ref_status_string",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UGVSigTgtMotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<SIMPLE_UGV>(120,40,1);
  m_pso_planner->initialize();


  m_mid_goal=make_float2(0,0);

  m_ref_v = 0.0f;
  m_ref_w = 0.0f;
  m_ref_theta = 0.0f;

  m_v_err_reset_ctt = 0;
  m_w_err_reset_ctt = 0;
  //Initialize the control message
  m_ref_msg.rows = 3;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;
}

UGVSigTgtMotionPlanner::~UGVSigTgtMotionPlanner()
{


  m_pso_planner->release();
  delete m_pso_planner;
}

void UGVSigTgtMotionPlanner::nf1_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
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
  m_pso_planner->m_eva.m_nf1_received = true;
}

void UGVSigTgtMotionPlanner::plan_call_back(const ros::TimerEvent&)
{

  ros::Time curr_t = ros::Time::now();
  cycle_initialized = false;

  cycle_process_based_on_status();

  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;


  for (UGV::UGVModel::State traj_s : m_traj)
  {
    ref_counter++;
    add_to_ref_msg(m_ref_msg,ref_counter,traj_s);

    if (ref_counter == next_ref_start_idx)
    {
      m_ref_v = traj_s.v;
      m_ref_w = traj_s.w;
      m_ref_theta = traj_s.theta;
    }

    cols++;
  }

  //std::cout<<m_ref_theta<<std::endl;

  m_ref_start_idx = next_ref_start_idx;

  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
  update_reference_log(m_ref_msg,curr_t);

  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();
#ifdef SHOW_PC
  plot_trajectory(m_traj);
#endif
  m_plan_cycle++;
}



void UGVSigTgtMotionPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_goal_received = true;
  m_goal.s.p.x = msg->pose.position.x;
  m_goal.s.p.y = msg->pose.position.y;

  double phi,theta,psi;

  tf::Quaternion q( msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);


  m_goal.s.theta = psi;
  m_goal.id++;
}

void UGVSigTgtMotionPlanner::mid_goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_mid_goal_received = true;
  m_mid_goal.x = msg->pose.position.x;
  m_mid_goal.y = msg->pose.position.y;
}
//=====================================
void UGVSigTgtMotionPlanner::do_start()
{
if (m_slam_odo_received && m_raw_odo_received && m_received_map && m_goal_received)
  m_status = UGV::NORMAL;
}
//=====================================
void UGVSigTgtMotionPlanner::do_normal()
{
  cycle_init();
  std::cout<<"NORMAL"<<std::endl;

  //Planning
  m_pso_planner->m_eva.m_pure_turning = false;
  m_pso_planner->m_eva.setTarget(m_goal);
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj);

  //Goto: Braking
  if (m_pso_planner->result.collision)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }
  else
  {
    //Goto: Stuck
    if(is_stuck(m_traj,m_goal.s))
    {
      m_status = UGV::STUCK;

      m_stuck_goal = m_goal;
      m_stuck_goal.s = m_pso_planner->m_model.get_ini_state();
      if (m_mid_goal_received)
      {
        float2 diff = m_mid_goal - m_stuck_goal.s.p;
        m_stuck_goal.s.theta = un_in_pi(atan2f(diff.y,diff.x),m_stuck_goal.s.theta);
      }
      else
      {
        m_stuck_goal.s.theta += 0.5*M_PI;
      }

    }

    //Goto: Pos_reached
    if(is_pos_reached(m_pso_planner->m_model.get_ini_state(),m_goal.s))
    {
      m_status = UGV::POS_REACHED;
    }
  }


}
//=====================================
void UGVSigTgtMotionPlanner::do_stuck()
{
  cycle_init();
  std::cout<<"STUCK"<<std::endl;

  //Planning
  m_pso_planner->m_eva.m_pure_turning = true;
  m_pso_planner->m_eva.setTarget(m_stuck_goal);
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj);

  //Goto: Normal (Finished turning)
  if (is_heading_reached(m_pso_planner->m_model.get_ini_state(),m_stuck_goal.s))
  {
    m_status = UGV::NORMAL;
  }

  //Goto: Normal (Found excape trajectory)
  if (!m_mid_goal_received)
  {
    std::vector<UGV::UGVModel::State> tmp_traj;
    m_pso_planner->m_eva.m_pure_turning = false;
    m_pso_planner->m_eva.setTarget(m_goal);
    calculate_trajectory<SIMPLE_UGV>(m_pso_planner, tmp_traj);
    if(!is_stuck_instant_horizon(tmp_traj,m_goal.s))
    {
      m_status = UGV::NORMAL;
      m_traj = tmp_traj;
    }
  }

  //Goto: Normal (New target)
  if (m_goal.id != m_pso_planner->m_eva.m_goal.id)
  {
    m_status = UGV::NORMAL;
  }
}
//=====================================
void UGVSigTgtMotionPlanner::do_emergent()
{
  cycle_init();
}
//=====================================
void UGVSigTgtMotionPlanner::do_braking()
{
  cycle_init();
  std::cout<<"BRAKING"<<std::endl;

  //Planning
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  //Goto: Normal
  if (m_plan_cycle - m_braking_start_cycle >= 10)
  {
     m_status = UGV::NORMAL;
  }

}
//=====================================
void UGVSigTgtMotionPlanner::do_pos_reached()
{
  cycle_init();
  std::cout<<"POS_REACHED"<<std::endl;

  // Planning
  m_pso_planner->m_eva.m_pure_turning = true;
  calculate_trajectory<SIMPLE_UGV>(m_pso_planner, m_traj);
  if(is_heading_reached(m_pso_planner->m_model.get_ini_state(),m_goal.s))
  {
    m_status = UGV::FULLY_REACHED;
  }

  //Goto: Normal (New target)
  if (m_goal.id != m_pso_planner->m_eva.m_goal.id)
  {
    m_status = UGV::NORMAL;
  }
}
//=====================================
void UGVSigTgtMotionPlanner::do_fully_reached()
{
  cycle_init();
  std::cout<<"FULLY_REACHED"<<std::endl;

  // Planing
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  //Goto: Normal (New target)
  if (m_goal.id != m_pso_planner->m_eva.m_goal.id)
  {
    m_status = UGV::NORMAL;
  }
}
//=====================================
void UGVSigTgtMotionPlanner::cycle_init()
{
  if (cycle_initialized)
    return;

  cycle_initialized = true;
  double phi,theta,psi;

  tf::Quaternion q(m_slam_odo.pose.pose.orientation.x,
                   m_slam_odo.pose.pose.orientation.y,
                   m_slam_odo.pose.pose.orientation.z,
                   m_slam_odo.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);

  if (m_status != UGV::START)
    psi = m_ref_theta;

  UGV::UGVModel::State s = predict_state(m_slam_odo,psi,m_ref_start_idx);


  float v_err = m_ref_v-m_raw_odo.twist.twist.linear.x;
  float w_err = m_ref_w-m_raw_odo.twist.twist.angular.z;

  if (fabs(v_err) > 1.0 )
    m_v_err_reset_ctt++;
  else
    m_v_err_reset_ctt = 0;

  if (fabs(w_err) > 1.0)
    m_w_err_reset_ctt++;
  else
    m_w_err_reset_ctt = 0;

  if (m_v_err_reset_ctt > 5)
  {
    std::cout<<"------Reset v------"<<std::endl;
    s.v = m_raw_odo.twist.twist.linear.x + sgn<float>(v_err)*0.5;
    m_v_err_reset_ctt = 0;
  }
  else
  {
    s.v = m_ref_v;
  }

  if (m_w_err_reset_ctt > 5)
  {
    std::cout<<"------Reset w------"<<std::endl;
    s.w = m_raw_odo.twist.twist.angular.z + sgn<float>(w_err)*0.5;
    m_w_err_reset_ctt = 0;
  }
  else
  {
    s.w = m_ref_w;
  }

  m_pso_planner->m_model.set_ini_state(s);
  m_traj = m_pso_planner->generate_trajectory();

  //Publish the ref status string for loging
  std_msgs::String msg;
  std::stringstream ss;
  ss << "STT: " << m_status<<", CST:"<< m_pso_planner->result.best_cost<<", COL:"<<m_pso_planner->result.collision<<std::endl;
  msg.data = ss.str();
  m_status_pub.publish(msg);
}
