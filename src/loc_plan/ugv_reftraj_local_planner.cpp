#include "loc_plan/ugv_reftraj_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <std_msgs/String.h>
#include <cpc_motion_planning/smooth_plan_request.h>
#include <cpc_motion_planning/collision_check.h>

UGVRefTrajMotionPlanner::UGVRefTrajMotionPlanner():
  m_goal_received(false),
  m_hybrid_path_received(false),
  cycle_initialized(false),
  m_ok_to_send_request(true),
  m_braking_start_cycle(0)
{
  m_goal_sub = m_nh.subscribe("/move_base_simple/goal",1,&UGVRefTrajMotionPlanner::goal_call_back, this);
  m_hybrid_path_sub = m_nh.subscribe("/hybrid_path",1,&UGVRefTrajMotionPlanner::hybrid_path_call_back,this);

  m_collision_check_client = m_nh.serviceClient<cpc_motion_planning::collision_check>("collision_check");
  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_status_pub = m_nh.advertise<std_msgs::String>("ref_status_string",1);
  m_tgt_reached_pub = m_nh.advertise<std_msgs::Int32>("target_reached",1);
  m_stuck_plan_request_pub = m_nh.advertise<cpc_motion_planning::smooth_plan_request>("/smooth_plan_request",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UGVRefTrajMotionPlanner::plan_call_back, this);


  m_ref_v = 0.0f;
  m_ref_w = 0.0f;
  m_ref_theta = 0.0f;

  m_v_err_reset_ctt = 0;
  m_w_err_reset_ctt = 0;
  m_tht_err_reset_ctt = 0;
  m_stuck_recover_path.request_ctt = -1;
  //Initialize the control message
  m_ref_msg.rows = 5;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;
}

UGVRefTrajMotionPlanner::~UGVRefTrajMotionPlanner()
{

}


void UGVRefTrajMotionPlanner::plan_call_back(const ros::TimerEvent&)
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


void UGVRefTrajMotionPlanner::goal_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  m_goal_received = true;

  double phi,theta,psi;

  tf::Quaternion q( msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);

  m_goal_pose.p.x = msg->pose.position.x;
  m_goal_pose.p.y = msg->pose.position.y;
  m_goal_pose.theta = theta;

  request_path();
}


void UGVRefTrajMotionPlanner::hybrid_path_call_back(const cpc_motion_planning::path::ConstPtr &msg)
{

  if (msg->request_ctt != m_plan_request_cycle)
    return;

  m_ok_to_send_request = true;
  m_hybrid_path_received = true;
  m_stuck_recover_path = *msg;
  m_planner.set_path_cell(m_stuck_recover_path);
}

void UGVRefTrajMotionPlanner::request_path()
{
  if (!m_ok_to_send_request)
    return;

  cpc_motion_planning::smooth_plan_request rq_msg;
  rq_msg.request_ctt = m_plan_cycle;
  rq_msg.start_x = m_current_pose.p.x;
  rq_msg.start_y = m_current_pose.p.y;
  rq_msg.start_theta = m_current_pose.theta;
  rq_msg.goal_x = m_goal_pose.p.x;
  rq_msg.goal_y = m_goal_pose.p.y;
  rq_msg.goal_theta = m_goal_pose.theta;
  m_stuck_plan_request_pub.publish(rq_msg);
  m_plan_request_cycle = m_plan_cycle;
  m_ok_to_send_request = false;
}

bool UGVRefTrajMotionPlanner::get_collision()
{
  cpc_motion_planning::collision_check srv;
  srv.request.collision_checking_path = m_planner.get_collision_checking_path();
  m_collision_check_client.call(srv);
  return srv.response.collision;
}

//=====================================
void UGVRefTrajMotionPlanner::do_start()
{
  std::cout<<"START"<<std::endl;
  std::cout<<m_slam_odo_received<<m_raw_odo_received<<m_received_map<<m_goal_received<<m_hybrid_path_received<<std::endl;
  if (m_slam_odo_received && m_raw_odo_received && m_received_map && m_goal_received && m_hybrid_path_received)
  {
    m_status = UGV::NORMAL;
  }
  else
  {
    if (m_slam_odo_received)
    {
      m_ref_theta = get_heading(m_slam_odo);
      m_current_pose.theta = m_ref_theta;
      m_current_pose.p.x = m_slam_odo.pose.pose.position.x;
      m_current_pose.p.y = m_slam_odo.pose.pose.position.y;
    }
  }
}
//=====================================
void UGVRefTrajMotionPlanner::do_normal()
{
  cycle_init();
  std::cout<<"NORMAL"<<std::endl;

  if (m_stuck_recover_path.actions.size() == 0)
  {
    // the response is empty, go to the full stuck sub mode
    full_stop_trajectory(m_traj,m_current_pose);
    return;
  }

  // do the recover path following
  bool finished = m_planner.calculate_trajectory(m_current_pose, m_edt_map, m_traj);

  if (m_planner.should_braking())
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
    return;
  }

  if (finished)
  {
    std::cout<<"request_from_finished"<<std::endl;
    request_path();
    return;
  }

  if(get_collision())
  {
    std::cout<<"request_from_collision"<<std::endl;
    request_path();
    return;
  }

  if(is_stuck(m_traj,m_goal_pose) || is_stuck_lowpass(m_current_pose))
  {
    std::cout<<"request_from_stuck"<<std::endl;
    request_path();
    return;
  }

  if(m_plan_cycle-m_plan_request_cycle>=50)
  {
    std::cout<<"request_from_timer"<<std::endl;
    request_path();
    return;
  }
}
//=====================================
void UGVRefTrajMotionPlanner::do_stuck()
{
  cycle_init();

}
//=====================================
void UGVRefTrajMotionPlanner::do_emergent()
{
  cycle_init();
}
//=====================================
void UGVRefTrajMotionPlanner::do_braking()
{
  cycle_init();
  std::cout<<"BRAKING"<<std::endl;

  //Planning
  full_stop_trajectory(m_traj,m_current_pose);

  //Goto: stuck - full stuck
  if (m_plan_cycle - m_braking_start_cycle >= 10)
  {
     m_status = UGV::STUCK;
     m_full_start_cycle = m_plan_cycle;
  }

}
//=====================================
void UGVRefTrajMotionPlanner::do_pos_reached()
{
  cycle_init();
  std::cout<<"POS_REACHED"<<std::endl;


}
//=====================================
void UGVRefTrajMotionPlanner::do_fully_reached()
{
  cycle_init();
  std::cout<<"FULLY_REACHED"<<std::endl;

}
//=====================================
void UGVRefTrajMotionPlanner::do_dropoff()
{
  cycle_init();
  std::cout<<"DROPOFF"<<std::endl;

}
//=====================================
void UGVRefTrajMotionPlanner::cycle_init()
{
  if (cycle_initialized)
    return;

  cycle_initialized = true;
  bool is_heading_ref;
  float psi = select_mes_ref_heading(is_heading_ref,get_heading(m_slam_odo), m_ref_theta, m_tht_err_reset_ctt, 0.25f);

  UGV::UGVModel::State s = predict_state(m_slam_odo,psi,m_ref_start_idx,is_heading_ref);

  s.v = select_mes_ref(m_raw_odo.twist.twist.linear.x, m_ref_v, m_v_err_reset_ctt);
  s.w = select_mes_ref(m_raw_odo.twist.twist.angular.z, m_ref_w, m_w_err_reset_ctt);

  m_current_pose = s;
  full_stop_trajectory(m_traj,m_current_pose);

  //Publish the ref status string for loging
  std_msgs::String msg;
  std::stringstream ss;
  ss << "STT: " << m_status<<std::endl;
  msg.data = ss.str();
  m_status_pub.publish(msg);
}
