#include <loc_plan/ugv_base_local_planner.h>

UGVLocalMotionPlanner::UGVLocalMotionPlanner():
  m_received_map(false),
  m_raw_odo_received(false),
  m_slam_odo_received(false),
  m_edt_map(nullptr),
  m_status(UGV::START),
  m_stuck_pbty(0.0f)
{
  m_map_sub = m_nh.subscribe("/edt_map", 1, &UGVLocalMotionPlanner::map_call_back, this);
  m_raw_odom_sub = m_nh.subscribe("/raw_odom", 1, &UGVLocalMotionPlanner::raw_odo_call_back, this);
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &UGVLocalMotionPlanner::slam_odo_call_back, this);
  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_traj_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_traj_pnt_cld->header.frame_id = "/world";
}

UGVLocalMotionPlanner::~UGVLocalMotionPlanner()
{
  if (m_edt_map)
  {
    m_edt_map->free_device();
    delete m_edt_map;
  }
}

void UGVLocalMotionPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
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

void UGVLocalMotionPlanner::raw_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_raw_odo_received = true;
  m_raw_odo = *msg;
}

void UGVLocalMotionPlanner::slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_slam_odo_received = true;
  m_slam_odo = *msg;
}

UGV::UGVModel::State UGVLocalMotionPlanner::predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx)
{
  UGV::UGVModel::State s;
  s.p.x = odom.pose.pose.position.x;
  s.p.y = odom.pose.pose.position.y;
  s.s = 0;
  s.theta = psi;
#ifdef PRED_STATE
  // Find the most related cmd
  while (m_cmd_q.size()>0)
  {
    if (m_cmd_q.front().t.toSec() <= odom.header.stamp.toSec())
      m_cmd_q.pop_front();
    else
      break;
  }

  for (const CmdLog &tmp : m_cmd_q)
  {
    if (tmp.id < ref_start_idx)
    {
      s.p.x = s.p.x + tmp.v*cos(s.theta)*PSO::PSO_CTRL_DT;
      s.p.y = s.p.y + tmp.v*sin(s.theta)*PSO::PSO_CTRL_DT;
      s.theta = s.theta + tmp.w*PSO::PSO_CTRL_DT;
    }
    else
    {
      break;
    }
  }
#endif
  return s;
}

void UGVLocalMotionPlanner::update_reference_log(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t)
{
#ifdef PRED_STATE
  if(m_cmd_q.empty())
  {
    load_into_queue(ref, curr_t);
  }
  else
  {
    int diff = ref.ids[0] - m_cmd_q.front().id;
    while(static_cast<int>(m_cmd_q.size()) > diff && !m_cmd_q.empty())
    {
      m_cmd_q.pop_back();
    }
    load_into_queue(ref, curr_t);
  }
#endif
}

#ifdef PRED_STATE
void UGVLocalMotionPlanner::load_into_queue(const cpc_motion_planning::ref_data &ref, const ros::Time &curr_t)
{
  for (int i=0; i<ref.cols; i++)
  {
    CmdLog tmp;
    tmp.t = curr_t + ros::Duration((i+1)*PSO::PSO_CTRL_DT);

    tmp.id = ref.ids[i];
    tmp.v = ref.data[i*ref.rows];
    tmp.w = ref.data[i*ref.rows + 1];
    //std::cout<<"id: "<<ref.ids[i]<<", "<<tmp.t<<std::endl;
    m_cmd_q.push_back(tmp);
  }
}
#endif

void UGVLocalMotionPlanner::add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter, const UGV::UGVModel::State &traj_s)
{
  ref_msg.ids.push_back(ref_counter);
  ref_msg.data.push_back(traj_s.v);
  ref_msg.data.push_back(traj_s.w);
}

#ifdef SHOW_PC
void UGVLocalMotionPlanner::plot_trajectory(const std::vector<UGV::UGVModel::State> &traj)
{
  for (UGV::UGVModel::State traj_s : traj)
  {
    pcl::PointXYZ clrP;
    clrP.x = traj_s.p.x;
    clrP.y = traj_s.p.y;
    clrP.z = 0;
    m_traj_pnt_cld->points.push_back(clrP);
  }
  m_traj_pub.publish(m_traj_pnt_cld);
  m_traj_pnt_cld->clear();
}
#endif

void UGVLocalMotionPlanner::cycle_process_based_on_status()
{
  switch (m_status) {
  case UGV::START:
  {
    do_start();
    break;
  }
  case UGV::NORMAL:
  {
    do_normal();
    break;
  }
  case UGV::STUCK:
  {
    do_stuck();
    break;
  }
  case UGV::EMERGENT:
  {
    do_emergent();
    break;
  }
  case UGV::BRAKING:
  {
    do_braking();
    break;
  }
  case UGV::POS_REACHED:
  {
    do_pos_reached();
    break;
  }
  case UGV::FULLY_REACHED:
  {
    do_fully_reached();
    break;
  }
  }
}

bool UGVLocalMotionPlanner::is_stuck(const std::vector<UGV::UGVModel::State> &traj, const float &best_cost)
{
  // update stuck probability
  if (is_stuck_instant(traj,best_cost))
      m_stuck_pbty +=0.15f;
  else
      m_stuck_pbty *=0.8f;

  if (m_stuck_pbty > 1)
  {
      m_stuck_pbty = 0;
      return true;
  }
  else
  {
      return false;
  }
}

bool UGVLocalMotionPlanner::is_stuck_instant(const std::vector<UGV::UGVModel::State> &traj, const float &best_cost)
{
  if (m_status <= UGV::START)
      return false;

  bool far_from_tgt = false;
  bool no_turning = false;
  bool no_moving_intention = false;

  // check is it far from target
  // TODO: make a better checking condition
  if (best_cost > 10)
      far_from_tgt = true;

  // check whether the vehicle is about to move
  float max_dist = 0;
  float dist;
  UGV::UGVModel::State ini_s = traj[0];
  float2 p_shift = make_float2(0,0);
  float max_turn = 0;
  float turn;
  for (UGV::UGVModel::State s : traj)
  {
      p_shift = ini_s.p - s.p;
      dist = sqrtf(dot(p_shift,p_shift));
      if (dist > max_dist)
          max_dist = dist;

      turn = fabsf(s.theta - ini_s.theta);
      if (turn > max_turn)
          max_turn = turn;
  }
  if (max_dist < 0.4f)
      no_moving_intention = true;

  if (max_turn < 0.25f)
    no_turning = true;

  return far_from_tgt && no_turning && no_moving_intention;
}

bool UGVLocalMotionPlanner::is_stuck_instant_horizon(const std::vector<UGV::UGVModel::State> &traj, const float &best_cost)
{
  if (m_status <= UGV::START)
      return false;

  bool far_from_tgt = false;
  bool no_moving_intention = false;

  // check is it far from target
  // TODO: make a better checking condition
  if (best_cost > 10)
      far_from_tgt = true;

  // check whether the vehicle is about to move
  float max_dist = 0;
  float dist;
  UGV::UGVModel::State ini_s = traj[0];
  float2 p_shift = make_float2(0,0);

  for (UGV::UGVModel::State s : traj)
  {
      p_shift = ini_s.p - s.p;
      dist = sqrtf(dot(p_shift,p_shift));
      if (dist > max_dist)
          max_dist = dist;
  }
  if (max_dist < 0.4f)
      no_moving_intention = true;


  return far_from_tgt && no_moving_intention;
}

