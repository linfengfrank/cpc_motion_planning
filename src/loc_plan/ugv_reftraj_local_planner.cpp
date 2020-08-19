#include "loc_plan/ugv_reftraj_local_planner.h"
#include "tf/tf.h"
#include <chrono>

//#define HDL_STUCK
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
  m_ref_theta = 0.0f;

  m_v_err_reset_ctt = 0;
  m_w_err_reset_ctt = 0;
  m_tht_err_reset_ctt = 0;
  //Initialize the control message
  m_ref_msg.rows = 5;
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
      m_ref_theta = traj_s.theta;
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

    float end_theta = m_loc_lines[0].front().tht;
    m_tgt.s.theta = end_theta;
    m_pso_planner->m_eva.setTarget(m_tgt);
    calculate_trajectory<REF_UGV>(m_pso_planner, m_traj);

    if(is_heading_reached(m_pso_planner->m_model.get_ini_state(),m_tgt.s))
    {
      m_status = UGV::NORMAL;
    }
  }
  else
  {
    if (m_slam_odo_received)
      m_ref_theta = get_heading(m_slam_odo);
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
#ifdef HDL_STUCK
    //Goto: Stuck
    if(is_stuck(m_traj,m_tgt.s))
    {
      //Find the target point as the next waypoint that is
      //1. As a turning point
      //2. On the global waypoint list
      waypoint astar_tgt = m_loc_lines[m_path_idx].back().b;
      for (size_t i=m_path_idx;i<m_loc_lines.size();i++)
      {
        if (m_loc_lines[i].back().b.id > 0)
        {
          astar_tgt = m_loc_lines[i].back().b;
          break;
        }
      }

      // Construct and call the services
      cpc_motion_planning::astar_service srv;
      call_a_star_service(srv,astar_tgt);

      // If the target cannot be reached, go to the next target
      if(!srv.response.reachable)
      {
        int tgt_glb_wp_id = min(astar_tgt.id+1, m_glb_wps.size());
        astar_tgt = m_glb_wps[tgt_glb_wp_id];
        call_a_star_service(srv,astar_tgt);
      }

      // Get the A star path
      std::vector<waypoint> wps;
      for(geometry_msgs::Pose pose : srv.response.wps)
      {
        wps.push_back(waypoint(make_float2(pose.position.x,pose.position.y),-1));
      }

      // Add in the rest of the original plan
      float2 diff = wps.back().p - astar_tgt.p;
      if (sqrt(dot(diff,diff))>0.5f)
        wps.push_back(astar_tgt);

      for (size_t i = astar_tgt.id+1; i<m_glb_wps.size(); i++)
      {
        wps.push_back(m_glb_wps[i]);
      }

      update_path(wps);
      show_path(wps);
      m_status = UGV::STUCK;

    }
#endif
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

  std::cout<<"STUCK"<<std::endl;
  // Planning
  m_pso_planner->m_eva.m_pure_turning = true;

  float end_theta = m_loc_lines[0].front().tht;
  m_tgt.s.theta = end_theta;
  m_pso_planner->m_eva.setTarget(m_tgt);
  calculate_trajectory<REF_UGV>(m_pso_planner, m_traj);

  if(is_heading_reached(m_pso_planner->m_model.get_ini_state(),m_tgt.s))
  {
    m_status = UGV::NORMAL;
  }
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
  if (m_path_idx + 1 < m_loc_lines.size())
  {
    end_theta = m_loc_lines[m_path_idx+1].front().tht;
  }
  else
  {
    end_theta = m_loc_lines[m_path_idx].back().tht;
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
  full_stop_trajectory(m_traj,m_pso_planner->m_model.get_ini_state());

  if (m_path_idx + 1 < m_loc_lines.size())
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
  float psi = select_mes_ref(get_heading(m_slam_odo), m_ref_theta, m_tht_err_reset_ctt, true, 0.25f);

  UGV::UGVModel::State s = predict_state(m_slam_odo,psi,m_ref_start_idx);

  s.v = select_mes_ref(m_raw_odo.twist.twist.linear.x, m_ref_v, m_v_err_reset_ctt);
  s.w = select_mes_ref(m_raw_odo.twist.twist.angular.z, m_ref_w, m_w_err_reset_ctt);

  calculate_ref_traj(s.p);
  m_pso_planner->m_model.set_ini_state(s);
  m_traj = m_pso_planner->generate_trajectory();
}

void UGVRefTrajMotionPlanner::load_ref_lines()
{
  if(!(m_slam_odo_received && m_raw_odo_received && m_received_map))
    return;

  std::ifstream corridor_file;
   corridor_file.open("/home/ugv/yzchen_ws/scout_ws/src/cpc_src/core_modules/cpc_motion_planning/cfg/in2.txt");
  //  corridor_file.open("/home/yzchen/CODE/higgs/higgs_ugv/src/autowalker_wrapper/cpc_src/core_modules/cpc_motion_planning/cfg/in.txt");
  float data[2];
  std::vector<waypoint> wps;
  float2 vehicle_pos = make_float2(m_slam_odo.pose.pose.position.x,m_slam_odo.pose.pose.position.y);
  int wp_id = 0;
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
          wps.push_back(waypoint(vehicle_pos,wp_id++));
        }
      }
      wps.push_back(waypoint(make_float2(data[0],data[1]),wp_id++));
      std::cout<<data[0]<<" "<<data[1]<<std::endl;
    }
    else
    {
      break;
    }
  }
  m_glb_wps = wps;
  update_path(wps);
  show_path(wps);
}

void UGVRefTrajMotionPlanner::calculate_ref_traj(float2 c)
{
  std::vector<float2> &path = m_loc_paths[m_path_idx];
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


