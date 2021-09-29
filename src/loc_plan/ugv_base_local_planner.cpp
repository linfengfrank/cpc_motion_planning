#include <loc_plan/ugv_base_local_planner.h>

UGVLocalMotionPlanner::UGVLocalMotionPlanner():
  m_received_map(false),
  m_raw_odo_received(false),
  m_slam_odo_received(false),
  m_edt_map(nullptr),
  m_create_host_edt(false),
  m_status(UGV::START),
  m_stuck_pbty(0.0f),
  m_lowpass_stuck_pbty(0.0f),
  m_lowpass_v(0.0f),
  m_lowpass_w(0.0f)
{
  // Initialize subscribers
  m_map_sub = m_nh.subscribe("/edt_map", 1, &UGVLocalMotionPlanner::map_call_back, this);
  m_raw_odom_sub = m_nh.subscribe("/raw_odom", 1, &UGVLocalMotionPlanner::raw_odo_call_back, this);
#ifdef ADD_DELAY
  m_sub.subscribe(m_nh, "/slam_odom", 1);
  m_seq = new message_filters::TimeSequencer<nav_msgs::Odometry> (m_sub, ros::Duration(0.2), ros::Duration(0.01), 10);
  m_seq->registerCallback(&UGVLocalMotionPlanner::slam_odo_call_back, this);
#else
  m_slam_odom_sub = m_nh.subscribe("/slam_odom", 1, &UGVLocalMotionPlanner::slam_odo_call_back, this);
#endif

  // Initialize publishers
  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_simulate_traj_pub = m_nh.advertise<PointCloud> ("simulated_traj", 1);

  // Initialize point cloud to show the trajectory
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
    // Construct and initialize the edt map
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_edt_map = new EDTMap(origin,msg->width,edt_map_size);
    m_edt_map->m_create_host_cpy = m_create_host_edt;
    m_edt_map->setup_device();
  }
  else
  {
    // Set the edt map's origin location
    m_edt_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_edt_map->m_grid_step = msg->width;
  }

  // Copy map data to the device
  CUDA_MEMCPY_H2D(m_edt_map->m_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));

  // Copy map data to the host
  if (m_create_host_edt)
    memcpy(m_edt_map->m_hst_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));
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

UGV::UGVModel::State UGVLocalMotionPlanner::predict_state(const nav_msgs::Odometry &odom, const double &psi, const int &ref_start_idx, bool is_heading_ref)
{
  UGV::UGVModel::State s;
  s.p.x = odom.pose.pose.position.x;
  s.p.y = odom.pose.pose.position.y;
  s.s = 0;
  s.theta = psi;
#ifdef PRED_STATE
  // Find the most related cmd in time
  while (m_cmd_q.size()>0)
  {
    if (m_cmd_q.front().t.toSec() <= odom.header.stamp.toSec())
      m_cmd_q.pop_front();
    else
      break;
  }

  // Use the cmd to perform forward simulation to predicte the state until the ref_start_idx
  for (const CmdLog &tmp : m_cmd_q)
  {
    if (tmp.id < ref_start_idx)
    {
      if (!is_heading_ref)
      {
        s.p.x = s.p.x + tmp.v*cos(s.theta)*PSO::PSO_CTRL_DT;
        s.p.y = s.p.y + tmp.v*sin(s.theta)*PSO::PSO_CTRL_DT;
        s.theta = s.theta + tmp.w*PSO::PSO_CTRL_DT;
      }
      else
      {
        s.p.x = s.p.x + tmp.v*cos(tmp.theta)*PSO::PSO_CTRL_DT;
        s.p.y = s.p.y + tmp.v*sin(tmp.theta)*PSO::PSO_CTRL_DT;
      }
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
    // queue is empty, just directly add ref into queue
    load_into_queue(ref, curr_t);
  }
  else
  {
    // queue not empty, remove the original content until no more duplicated cmd id
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
    tmp.theta = ref.data[i*ref.rows + 2];
    //std::cout<<"id: "<<ref.ids[i]<<", "<<tmp.t<<std::endl;
    m_cmd_q.push_back(tmp);
  }
}
#endif

void UGVLocalMotionPlanner::add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter, const UGV::UGVModel::State &traj_s, const ros::Time &t)
{
  ref_msg.ids.push_back(ref_counter);
  ref_msg.time.push_back(t.toSec());
  ref_msg.data.push_back(traj_s.v);
  ref_msg.data.push_back(traj_s.w);
  ref_msg.data.push_back(traj_s.theta);
  ref_msg.data.push_back(traj_s.p.x);
  ref_msg.data.push_back(traj_s.p.y);
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
  case UGV::DROPOFF:
  {
    do_dropoff();
    break;
  }
  }
}

bool UGVLocalMotionPlanner::is_stuck(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state)
{
  // update stuck probability
  if (is_stuck_instant(traj,tgt_state))
      m_stuck_pbty +=0.15f;
  else
      m_stuck_pbty *=0.8f;

  if (m_stuck_pbty > 1)
  {
    // Remeber to set all forms of stuck probability
    m_lowpass_stuck_pbty = 0;
    m_stuck_pbty = 0;
    return true;
  }
  else
  {
    return false;
  }
}

bool UGVLocalMotionPlanner::is_stuck_instant(const std::vector<UGV::UGVModel::State> &traj, const UGV::UGVModel::State &tgt_state)
{
  if (m_status <= UGV::START)
      return false;

  bool far_from_tgt = false;
  bool no_turning = false;
  bool no_moving_intention = false;

  // check is it far from target
  if (!is_pos_reached(traj[0],tgt_state))
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

bool UGVLocalMotionPlanner::is_stuck_lowpass(const UGV::UGVModel::State& s, const UGV::UGVModel::State &tgt_state)
{
  // Use a low pass filter on the current velocity
  m_lowpass_v = 0.8f*m_lowpass_v + 0.2f*s.v;
  m_lowpass_w = 0.8f*m_lowpass_w + 0.2f*s.w;

  // If the current linear and rotational speed are both very small, increase the stuck probability
  if (fabsf(m_lowpass_v) < 0.08f && fabsf(m_lowpass_w) < 0.08f && !is_pos_reached(s,tgt_state))
    m_lowpass_stuck_pbty +=0.1f;
  else
    m_lowpass_stuck_pbty *=0.8f;

  //std::cout<<"****"<<m_lowpass_v<<" "<<m_lowpass_w<<" "<<m_lowpass_stuck_pbty<<std::endl;

  if (m_lowpass_stuck_pbty > 1)
  {
    // Remeber to set all forms of stuck probability
    m_lowpass_stuck_pbty = 0;
    m_stuck_pbty = 0;
    return true;
  }
  else
  {
    return false;
  }
}

std::vector<UGV::UGVModel::State> UGVLocalMotionPlanner::simulate_from_current_state(const std::vector<UGV::UGVModel::State> &ref, const nav_msgs::Odometry &odom,
                                                              float yaw_ctrl_gain, float w_scale, float exam_time)
{
  std::vector<UGV::UGVModel::State> output;
  //Get the true state from odom information
  UGV::UGVModel::State s;
  s.p.x = odom.pose.pose.position.x;
  s.p.y = odom.pose.pose.position.y;
  s.s = 0;
  s.theta = get_heading(odom);

  //Simulating the tracking of the trajectory
  float dt = PSO::PSO_CTRL_DT;
  for (int i=0; i*dt<=exam_time && i < ref.size(); i++)
  {
    s.v = ref[i].v;
    // For the yaw, we have a controller, and a simulated slip (w_scale)
    // The controller shall be exactly the same as the controller in the cpc_ref_publisher pacakge (main.cpp)
    s.w = ref[i].w/w_scale + yaw_ctrl_gain * in_pi(ref[i].theta - s.theta);
    s.w= s.w> 0.6? 0.6:s.w;
    s.w= s.w< -0.6? -0.6:s.w;
    s.w *= w_scale;

    //s and theta
    s.s = s.s + s.v*dt;
    s.theta = s.theta + s.w*dt;

    // x and y
    s.p.x = s.p.x + (s.v*dt)*cos(s.theta + s.w*dt);
    s.p.y = s.p.y + (s.v*dt)*sin(s.theta + s.w*dt);

    output.push_back(s);
  }
  return output;
}

float UGVLocalMotionPlanner::find_traj_min_dist_to_obstacle(const std::vector<UGV::UGVModel::State> &traj)
{
  // Find the smallest distance to obstacle
  float min_dist = 1000;
  for (const UGV::UGVModel::State &s : traj)
  {
    //Get dist to obstacle for current state s
    float dist = get_dist_from_host_edt(s);
    if (dist < min_dist)
      min_dist = dist;
  }
  return min_dist;
}

// This function check whether there is a collision by simulating a tracking of m_traj from the
// true initial state (aka. consider the tracking error).
bool UGVLocalMotionPlanner::true_state_collision_exam(bool use_adrc, const nav_msgs::Odometry &odom, const std::vector<UGV::UGVModel::State> &ref,
                                                      float yaw_ctrl_gain, float safety_radius, float w_scale, float exam_time)
{
  //If it is in ADRC mode, no need to do the checking
  //directly return true (aka. no collision).
  if (use_adrc)
    return true;

  // Simulate the trajectory tracking process
  std::vector<UGV::UGVModel::State> sim_traj = simulate_from_current_state(ref,odom,yaw_ctrl_gain,w_scale,exam_time);

  // Find the smallest distance to obstacle
  float min_dist = find_traj_min_dist_to_obstacle(sim_traj);

#ifdef SHOW_PC
  for (UGV::UGVModel::State &s : sim_traj)
  {
    pcl::PointXYZ clrP;
    clrP.x = s.p.x;
    clrP.y = s.p.y;
    clrP.z = 0;
    m_traj_pnt_cld->points.push_back(clrP);
  }
  m_simulate_traj_pub.publish(m_traj_pnt_cld);
  m_traj_pnt_cld->clear();
#endif

  if (min_dist < safety_radius)
    return false;
  else
    return true;
}

