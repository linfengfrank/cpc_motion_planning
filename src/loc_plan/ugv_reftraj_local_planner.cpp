#include "loc_plan/ugv_reftraj_local_planner.h"
#include "tf/tf.h"
#include <chrono>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

UGVRefTrajMotionPlanner::UGVRefTrajMotionPlanner():
  m_goal_received(false),
  cycle_initialized(false),
  m_braking_start_cycle(0)
{
  m_goal_sub = m_nh.subscribe("/move_base_simple/goal",1,&UGVRefTrajMotionPlanner::goal_call_back, this);


  m_ref_pub = m_nh.advertise<cpc_motion_planning::ref_data>("ref_traj",1);
  m_vis_pub = m_nh.advertise<visualization_msgs::Marker>("path_viz",1);

  m_planning_timer = m_nh.createTimer(ros::Duration(PSO::PSO_REPLAN_DT), &UGVRefTrajMotionPlanner::plan_call_back, this);

  m_pso_planner = new PSO::Planner<REF_UGV>(100,30,2);
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

UGVRefTrajMotionPlanner::~UGVRefTrajMotionPlanner()
{


  m_pso_planner->release();
  delete m_pso_planner;
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
    }

    cols++;
  }

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
  load_ref_lines();
}
//---
void UGVRefTrajMotionPlanner::do_start()
{
if (m_slam_odo_received && m_raw_odo_received && m_received_map && m_goal_received)
  m_status = UGV::NORMAL;
}
//---
void UGVRefTrajMotionPlanner::do_normal()
{
  cycle_init();
  auto start = std::chrono::steady_clock::now();

  calculate_trajectory<REF_UGV>(m_pso_planner, m_traj);

  if (m_pso_planner->result.collision)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }
  else
  {
//    UGV::UGVModel::State tmp_goal = float3_to_goal_state(m_ref_traj.tarj[40]);
//    if(is_stuck(m_traj,tmp_goal))
//    {
//      m_status = UGV::STUCK;
//    }
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (NORMAL): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;
}
//---
void UGVRefTrajMotionPlanner::do_stuck()
{
  cycle_init();
  auto start = std::chrono::steady_clock::now();
  m_traj.clear();
  float dt = PSO::PSO_CTRL_DT;;
  for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
  {
    UGV::UGVModel::State s;
    s.v = 0;
    s.w = 0.5;
    m_traj.push_back(s);
  }

  std::vector<UGV::UGVModel::State> tmp_traj;
  calculate_trajectory<REF_UGV>(m_pso_planner, tmp_traj);
  UGV::UGVModel::State tmp_goal = float3_to_goal_state(m_ref_traj.tarj[40]);
  if(!is_stuck_instant_horizon(tmp_traj,tmp_goal))
  {
    m_status = UGV::NORMAL;
    m_traj = tmp_traj;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (STUCK): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;
}
//---
void UGVRefTrajMotionPlanner::do_emergent()
{
  cycle_init();
}
//---
void UGVRefTrajMotionPlanner::do_braking()
{
  cycle_init();
  auto start = std::chrono::steady_clock::now();
  m_traj.clear();
  float dt = PSO::PSO_CTRL_DT;;
  for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
  {
    UGV::UGVModel::State s;
    s.v = 0;
    s.w = 0;
    m_traj.push_back(s);
  }
  if (m_plan_cycle - m_braking_start_cycle >= 10)
  {
     m_status = UGV::NORMAL;
  }
  auto end = std::chrono::steady_clock::now();
  std::cout << "local planner (BRAKING): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "ms, " << m_pso_planner->result.best_cost
            << ", collision: " << m_pso_planner->result.collision<<std::endl;
}

void UGVRefTrajMotionPlanner::do_pos_reached()
{
  cycle_init();
}

void UGVRefTrajMotionPlanner::do_fully_reached()
{
  cycle_init();
}

void UGVRefTrajMotionPlanner::cycle_init()
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

  calculate_ref_traj(make_float3(s.p.x,s.p.y,s.theta));
  m_pso_planner->m_model.set_ini_state(s);
  m_traj = m_pso_planner->generate_trajectory();
}

void UGVRefTrajMotionPlanner::load_ref_lines()
{
  std::ifstream corridor_file;
  corridor_file.open("/home/sp/nndp/Learning_part/tripple_integrator/pso/in.txt");
  float data[6];
  std::vector<float2> wps;

  std::cout<<"Read in data"<<std::endl;
  while(1)
  {
    if (corridor_file>>data[0]>>data[1]>>data[2]>>data[3]>>data[4]>>data[5])
    {
      wps.push_back(make_float2(data[0],data[1]));
      std::cout<<data[0]<<" "<<data[1]<<" "<<data[2]<<" "<<data[3]<<" "<<data[4]<<" "<<data[5]<<std::endl;
    }
    else
    {
      break;
    }
  }

  std::vector<float3> tmp_wps;
  //Assign the heading
  for (size_t i = 0; i < wps.size(); i++)
  {
    if (i == 0)
    {
      float theta = atan2f(wps[i+1].y-wps[i].y, wps[i+1].x-wps[i].x);
      tmp_wps.push_back(make_float3(wps[i].x,wps[i].y,theta));
    }
    else if (i == wps.size()-1)
    {
      float theta = atan2f(wps[i].y-wps[i-1].y, wps[i].x-wps[i-1].x);
      theta = un_in_pi(theta,tmp_wps.back().z);
      tmp_wps.push_back(make_float3(wps[i].x,wps[i].y,theta));
    }
    else
    {
      float theta = atan2f(wps[i].y-wps[i-1].y, wps[i].x-wps[i-1].x);
      theta = un_in_pi(theta,tmp_wps.back().z);
      tmp_wps.push_back(make_float3(wps[i].x,wps[i].y,theta));

      theta = atan2f(wps[i+1].y-wps[i].y, wps[i+1].x-wps[i].x);
      theta = un_in_pi(theta,tmp_wps.back().z);
      tmp_wps.push_back(make_float3(wps[i].x,wps[i].y,theta));
    }
  }

  //Dense sample bewteen the waypoints
  m_path.clear();
  for (size_t i = 0; i < tmp_wps.size()-1; i++)
  {
    std::vector<float3> tmp_pol = interpol(tmp_wps[i],tmp_wps[i+1]);
    for (float3 pol : tmp_pol)
    {
      m_path.push_back(pol);
    }
  }

  //Parameterization against the length
  m_path_len.clear();
  for (size_t i = 0; i < m_path.size(); i++)
  {
    if (i == 0)
    {
      m_path_len.push_back(make_float3(0,0,0));
    }
    else
    {
      float3 delta = m_path[i]-m_path[i-1];
      float dist_xy = sqrtf(delta.x*delta.x + delta.y*delta.y);
      float dist_theta = fabsf(delta.z);
      float dist = sqrtf(dot(delta,delta));
      float3 pre_len = m_path_len.back();
      m_path_len.push_back(make_float3(pre_len.x+dist_xy, pre_len.y+dist_theta, pre_len.z+dist));
    }
  }

  // For visulization
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.g = 1.0;
  line_strip.color.a = 1.0;

  for (float2 p: wps)
  {
    geometry_msgs::Point pnt;
    pnt.x = p.x;
    pnt.y = p.y;
    pnt.z = 0;
    line_strip.points.push_back(pnt);
  }
  m_vis_pub.publish(line_strip);

}

void UGVRefTrajMotionPlanner::calculate_ref_traj(float3 c)
{
  //c=[x y theta]
  //Find nearest point
  float min_len = 1e6;
  size_t min_idx = 0;
  float3 delta;
  float len;
  float3 nearest_pnt;
  for (size_t i=0;i<m_path.size();i++)
  {
    delta = m_path[i]-c;
    delta.z = in_pi(delta.z)*0.2f;
    len = sqrtf(dot(delta,delta));
    if (len < min_len)
    {
      min_len = len;
      min_idx = i;
      nearest_pnt = m_path[i];
    }
  }

  //Make local ref
  size_t fin = min_idx;
  for (size_t i=min_idx; i<m_path.size(); i++)
  {
    float3 delta_len = m_path_len[i] - m_path_len[min_idx];
//    float3 delta_c = m_path[i] - m_path[min_idx];
//    delta_c.z = in_pi(delta_c.z);
 //   std::cout<<delta_len.x<<std::endl;

    if (delta_len.x < 5 && delta_len.y < 2.0)//
      fin = i;
    else
      break;
  }

  //Interplocation
  std::vector<float> x,y,tht;
  std::vector<float> tmp_s;
  for (size_t i=min_idx; i<= fin; i++)
  {
    x.push_back(m_path[i].x);
    y.push_back(m_path[i].y);
    tht.push_back(m_path[i].z);
    tmp_s.push_back(m_path_len[i].z);
//    std::cout<<":"<<x.back()<<" "<<y.back()<<" "<<tht.back()<<" "<<tmp_s.back()<<std::endl;
  }

  std::vector<float> si = interpol(tmp_s.front(),tmp_s.back(),41);

  std::vector<float> xi = interp1(tmp_s,x,si);
  std::vector<float> yi = interp1(tmp_s,y,si);
  std::vector<float> thti = interp1(tmp_s,tht,si);
  std::cout<<min_idx<<" "<<fin<<std::endl;
//  std::cout<<si.size()<<" "<<xi.size()<<" "<<yi.size()<<" "<<thti.size()<<std::endl;
  for (int i=0; i<41; i++)
  {
    m_ref_traj.tarj[i]=make_float3(xi[i],yi[i],thti[i]);
  }

  m_pso_planner->m_eva.setRef(m_ref_traj);
}

void UGVRefTrajMotionPlanner::linecirc_inter_dist(const float3 &seg_a, const float3 &seg_b, const float3 &circ_pos, float3 &closest, float &dist_v_len) const
{
  float3 seg_v=seg_b-seg_a;
  float3 pt_v=circ_pos-seg_a;
  float seg_v_len = sqrtf(dot(seg_v,seg_v));
  float3 seg_v_unit=seg_v/seg_v_len;
  float proj=dot(pt_v,seg_v_unit);
  float3 proj_v=seg_v_unit*proj;
  if (proj <=0)
    closest =seg_a;
  else if(proj>seg_v_len)
    closest =seg_b;
  else
    closest=proj_v+seg_a;

  dist_v_len = sqrtf(dot(circ_pos-closest,circ_pos-closest));
}

float3 UGVRefTrajMotionPlanner::calculate_unit_vector(const float3 &seg_a, const float3 &seg_b)
{
  float3 seg_v=seg_b-seg_a;
  float seg_v_len = sqrtf(dot(seg_v,seg_v));
  return seg_v/seg_v_len;
}

float UGVRefTrajMotionPlanner::calculate_length(const float3 &seg_a, const float3 &seg_b)
{
  float3 seg_v=seg_b-seg_a;
  return sqrtf(dot(seg_v,seg_v));
}


