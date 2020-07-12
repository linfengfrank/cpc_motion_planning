#include "loc_plan/ugv_sigtgt_local_planner.h"
#include "tf/tf.h"
#include <chrono>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

UGVSigTgtMotionPlanner::UGVSigTgtMotionPlanner():
  m_goal_received(false)
{
  m_goal_sub = m_nh.subscribe("/move_base_simple/goal",1,&UGVSigTgtMotionPlanner::goal_call_back, this);


  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UGVSigTgtMotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<SIMPLE_UGV>(120,40,1);
  m_pso_planner->initialize();




  m_ref_v = 0.0f;
  m_ref_w = 0.0f;

  m_v_err_reset_ctt = 0;
  m_w_err_reset_ctt = 0;
  //Initialize the control message
  m_ref_msg.rows = 2;
  m_plan_cycle = 0;
  m_ref_start_idx = 0;
}

UGVSigTgtMotionPlanner::~UGVSigTgtMotionPlanner()
{


  m_pso_planner->release();
  delete m_pso_planner;
}

void UGVSigTgtMotionPlanner::plan_call_back(const ros::TimerEvent&)
{
  if (!m_slam_odo_received || !m_raw_odo_received || !m_received_map || !m_goal_received)
    return;

  double phi,theta,psi;

  tf::Quaternion q(m_slam_odo.pose.pose.orientation.x,
                   m_slam_odo.pose.pose.orientation.y,
                   m_slam_odo.pose.pose.orientation.z,
                   m_slam_odo.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(phi, theta, psi);
#ifdef PRED_STATE
  ros::Time curr_t = ros::Time::now();
  UGV::UGVModel::State s = predict_state(m_slam_odo,psi,m_ref_start_idx);
#else
  UGV::UGVModel::State s;
  s.p.x = m_slam_odo.pose.pose.position.x;
  s.p.y = m_slam_odo.pose.pose.position.y;
  s.s = 0;
  s.theta = psi;
#endif


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

  //std::cout<<"v,w err: "<<v_err<<", "<<w_err<<std::endl;
  //  float yaw_diff = s.theta - m_goal.theta;
  //  yaw_diff = yaw_diff - floor((yaw_diff + M_PI) / (2 * M_PI)) * 2 * M_PI;



  m_pso_planner->m_model.set_ini_state(s);
  m_pso_planner->m_eva.setTarget(m_goal);
  auto start = std::chrono::steady_clock::now();
  m_pso_planner->plan(*m_edt_map);
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;

  std::cout<<s.p.x<<" "<<s.p.y<<" "<<s.s<<" "<<s.theta<<" "<<s.v<<" "<<s.w<<" "<<std::endl;
  std::cout<<"v,w err: "<<v_err<<", "<<w_err<<std::endl;
  std::cout<<"--------------"<<std::endl;

  std::vector<UGV::UGVModel::State> traj = m_pso_planner->generate_trajectory();

  int cols = 0;
  int ref_counter = m_ref_start_idx;
  int next_ref_start_idx = (m_plan_cycle+1)*PSO::PSO_REPLAN_CYCLE+PSO::PSO_PLAN_CONSUME_CYCLE;


  for (UGV::UGVModel::State traj_s : traj)
  {
    pcl::PointXYZ clrP;
    clrP.x = traj_s.p.x;
    clrP.y = traj_s.p.y;
    clrP.z = 0.0f;
    m_traj_pnt_cld->points.push_back(clrP);

    ref_counter++;
    m_ref_msg.ids.push_back(ref_counter);
    m_ref_msg.data.push_back(traj_s.v);
    m_ref_msg.data.push_back(traj_s.w);

    if (ref_counter == next_ref_start_idx)
    {
      m_ref_v = traj_s.v;
      m_ref_w = traj_s.w;
    }

    cols++;
  }

  m_ref_start_idx = next_ref_start_idx;

  m_ref_msg.cols = cols;
  m_ref_pub.publish(m_ref_msg);
#ifdef PRED_STATE
  update_reference_log(m_ref_msg,curr_t);
#endif
  m_ref_msg.data.clear();
  m_ref_msg.ids.clear();

  m_traj_pub.publish(m_traj_pnt_cld);
  m_traj_pnt_cld->clear();

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
}
