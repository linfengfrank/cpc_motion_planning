#include "loc_plan/ugv_reftraj_local_planner.h"
#include "tf/tf.h"
#include <chrono>
#include <cpc_motion_planning/astar_service.h>

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

  m_astar_client =  m_nh.serviceClient<cpc_motion_planning::astar_service>("/astar_service");

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
{
  cycle_init();
  std::cout<<"START"<<std::endl;

  // Planning
  m_pso_planner->m_eva.m_pure_turning = true;

  float end_theta = m_line_list[0].front().tht;
  m_tgt.s.theta = end_theta;
  m_pso_planner->m_eva.setTarget(m_tgt);
  calculate_trajectory<REF_UGV>(m_pso_planner, m_traj);

  if(is_heading_reached(m_pso_planner->m_model.get_ini_state(),m_tgt.s))
  {
    m_status = UGV::NORMAL;
  }
}
}
//---
void UGVRefTrajMotionPlanner::do_normal()
{
  cycle_init();
  std::cout<<"NORMAL"<<std::endl;

  //Planning
  m_pso_planner->m_eva.m_pure_turning = false;
  m_pso_planner->m_eva.setTarget(m_tgt);
  calculate_trajectory<REF_UGV>(m_pso_planner, m_traj);

  if (m_pso_planner->result.collision)
  {
    m_braking_start_cycle = m_plan_cycle;
    m_status = UGV::BRAKING;
    cycle_process_based_on_status();
  }
  else
  {
    //Goto: Stuck
    if(is_stuck(m_traj,m_tgt.s))
    {
      m_status = UGV::STUCK;
    }
    //Goto: Pos_reached
    if(is_pos_reached(m_pso_planner->m_model.get_ini_state(),m_tgt.s))
    {
      m_status = UGV::POS_REACHED;
    }
  }

}
//---
void UGVRefTrajMotionPlanner::do_stuck()
{
  cycle_init();
  cpc_motion_planning::astar_service srv;
  srv.request.target.position.x = m_path_list[m_path_idx].back().x;
  srv.request.target.position.y = m_path_list[m_path_idx].back().y;
  srv.request.target.position.z = 0;

  m_astar_client.call(srv);
//  auto start = std::chrono::steady_clock::now();
//  m_traj.clear();
//  float dt = PSO::PSO_CTRL_DT;;
//  for (float t=0.0f; t<PSO::PSO_TOTAL_T; t+=dt)
//  {
//    UGV::UGVModel::State s;
//    s.v = 0;
//    s.w = 0.5;
//    m_traj.push_back(s);
//  }

//  std::vector<UGV::UGVModel::State> tmp_traj;
//  calculate_trajectory<REF_UGV>(m_pso_planner, tmp_traj);
//  UGV::UGVModel::State tmp_goal = float3_to_goal_state(m_ref_traj.tarj[40]);
//  if(!is_stuck_instant_horizon(tmp_traj,tmp_goal))
//  {
//    m_status = UGV::NORMAL;
//    m_traj = tmp_traj;
//  }
//  auto end = std::chrono::steady_clock::now();
//  std::cout << "local planner (STUCK): "
//            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
//            << "ms, " << m_pso_planner->result.best_cost
//            << ", collision: " << m_pso_planner->result.collision<<std::endl;
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
  std::cout<<"POS_REACHED"<<std::endl;

  // Planning
  m_pso_planner->m_eva.m_pure_turning = true;

  float end_theta;
  if (m_path_idx + 1 < m_line_list.size())
  {
    end_theta = m_line_list[m_path_idx+1].front().tht;
  }
  else
  {
    end_theta = m_line_list[m_path_idx].back().tht;
  }
  m_tgt.s.theta = end_theta;
  m_pso_planner->m_eva.setTarget(m_tgt);
  calculate_trajectory<REF_UGV>(m_pso_planner, m_traj);

  if(is_heading_reached(m_pso_planner->m_model.get_ini_state(),m_tgt.s))
  {
    m_status = UGV::FULLY_REACHED;
  }
}

void UGVRefTrajMotionPlanner::do_fully_reached()
{
  cycle_init();
  std::cout<<"FULLY_REACHED"<<std::endl;

  // Planing
  full_stop_trajectory(m_traj);

  if (m_path_idx + 1 < m_line_list.size())
  {
    m_path_idx++;
    m_status = UGV::NORMAL;
  }
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

  calculate_ref_traj(s.p);
  m_pso_planner->m_model.set_ini_state(s);
  m_traj = m_pso_planner->generate_trajectory();
}

void UGVRefTrajMotionPlanner::load_ref_lines()
{
  if(!(m_slam_odo_received && m_raw_odo_received && m_received_map))
    return;

  std::ifstream corridor_file;
  corridor_file.open("/home/sp/nndp/Learning_part/tripple_integrator/pso/in.txt");
  float data[6];
  std::vector<float2> wps;
  float2 vehicle_pos = make_float2(m_slam_odo.pose.pose.position.x,m_slam_odo.pose.pose.position.y);
  std::cout<<"Read in data"<<std::endl;
  while(1)
  {
    if (corridor_file>>data[0]>>data[1])
    {
      if(wps.empty())
      {
        // Check whether the vehicle is far from the first wp
        float2 first_wp = make_float2(data[0],data[1]);
        if (sqrt(dot(vehicle_pos-first_wp,vehicle_pos-first_wp))>1)
        {
          wps.push_back(vehicle_pos);
        }
      }
      wps.push_back(make_float2(data[0],data[1]));
      std::cout<<data[0]<<" "<<data[1]<<std::endl;
    }
    else
    {
      break;
    }
  }

  // Use split & merge to identify the wps of sharp turning
  std::set<size_t> split_sharp_wp_ids;
  std::vector<size_t> split_wp_ids = findSplitCoords(wps, 3.0f);
  for (size_t i = 1; i< split_wp_ids.size()-1; i++)
  {
    line_seg l1(wps[split_wp_ids[i-1]],wps[split_wp_ids[i]]);
    line_seg l2(wps[split_wp_ids[i]],wps[split_wp_ids[i+1]]);

    if (fabsf(in_pi(l1.tht-l2.tht)) > 0.25*M_PI)
    {
      split_sharp_wp_ids.insert(split_wp_ids[i]);
      //std::cout<<split_wp_ids[i]<<std::endl;
    }
  }

  // Construct the line list
  std::vector<line_seg> lines;
  for (size_t i = 0; i < wps.size()-1; i++)
  {
    lines.push_back(line_seg(wps[i],wps[i+1]));
  }

  m_line_list.clear();
  std::vector<line_seg> tmp;
  for (size_t i = 0; i < lines.size()-1; i++)
  {
    tmp.push_back(lines[i]);
    if (fabsf(in_pi(lines[i].tht-lines[i+1].tht)) > 0.25*M_PI || split_sharp_wp_ids.count(i+1) != 0)
    {
      m_line_list.push_back(tmp);
      tmp.clear();
    }
  }
  tmp.push_back((lines.back()));
  m_line_list.push_back(tmp);

  // Construct the path
  m_path_list.clear();
  for (size_t i = 0; i < m_line_list.size(); i++)
  {
    std::vector<float2> path;
    for (size_t j = 0; j < m_line_list[i].size(); j++)
    {
      std::vector<float2> tmp_pol = interpol(m_line_list[i][j].a,m_line_list[i][j].b,0.8f);
      for (float2 pol : tmp_pol)
      {
        path.push_back(pol);
      }
    }
    m_path_list.push_back(path);
  }

  m_path_idx = 0;

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

void UGVRefTrajMotionPlanner::calculate_ref_traj(float2 c)
{
  std::vector<float2> &path = m_path_list[m_path_idx];
  //c=[x y theta]

  //Find nearest point
  float min_len = 1e6;
  size_t min_idx = 0;
  float2 delta;
  float len;
  float2 nearest_pnt;
  for (size_t i=0;i<path.size();i++)
  {
    delta = path[i]-c;
    len = sqrtf(dot(delta,delta));
    if (len < min_len)
    {
      min_len = len;
      min_idx = i;
      nearest_pnt = path[i];
    }
  }

  //Get the carrot point
  float2 carrot = nearest_pnt;
  for (size_t i=min_idx;i<path.size();i++)
  {
    delta = path[i]-c;
    len = sqrtf(dot(delta,delta));
    if (len < 2.8)
      carrot = path[i];
    else
      break;
  }

  //Set the carrot point
  m_tgt.s.p = carrot;
  float2 dist_err = m_tgt.s.p - c;
  m_tgt.s.theta = atan2f(dist_err.y,dist_err.x);





  //std::cout<<m_tgt.s.p.x<<" "<<m_tgt.s.p.y<<" "<<m_tgt.s.theta <<std::endl;
}

//void UGVRefTrajMotionPlanner::linecirc_inter_dist(const float3 &seg_a, const float3 &seg_b, const float3 &circ_pos, float3 &closest, float &dist_v_len) const
//{
//  float3 seg_v=seg_b-seg_a;
//  float3 pt_v=circ_pos-seg_a;
//  float seg_v_len = sqrtf(dot(seg_v,seg_v));
//  float3 seg_v_unit=seg_v/seg_v_len;
//  float proj=dot(pt_v,seg_v_unit);
//  float3 proj_v=seg_v_unit*proj;
//  if (proj <=0)
//    closest =seg_a;
//  else if(proj>seg_v_len)
//    closest =seg_b;
//  else
//    closest=proj_v+seg_a;

//  dist_v_len = sqrtf(dot(circ_pos-closest,circ_pos-closest));
//}

//float3 UGVRefTrajMotionPlanner::calculate_unit_vector(const float3 &seg_a, const float3 &seg_b)
//{
//  float3 seg_v=seg_b-seg_a;
//  float seg_v_len = sqrtf(dot(seg_v,seg_v));
//  return seg_v/seg_v_len;
//}

//float UGVRefTrajMotionPlanner::calculate_length(const float3 &seg_a, const float3 &seg_b)
//{
//  float3 seg_v=seg_b-seg_a;
//  return sqrtf(dot(seg_v,seg_v));
//}


