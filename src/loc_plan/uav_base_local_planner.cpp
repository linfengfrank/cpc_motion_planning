#include <loc_plan/uav_base_local_planner.h>

UAVLocalMotionPlanner::UAVLocalMotionPlanner():
  m_stuck_pbty(0.0f),
  m_received_map(false),
  m_pose_received(false),
  m_edt_map(nullptr),
  m_fly_status(UAV::AT_GROUND),
  m_hst_map(nullptr),
  m_use_hst_map(false)
{
  m_map_sub = m_nh.subscribe("edt_map", 1, &UAVLocalMotionPlanner::map_call_back, this);
  m_pose_sub = m_nh.subscribe("/ground_truth/state", 1, &UAVLocalMotionPlanner::vehicle_pose_call_back, this);
#ifdef SHOWPC
  m_traj_pub = m_nh.advertise<PointCloud> ("pred_traj", 1);
  m_traj_pnt_cld = PointCloud::Ptr(new PointCloud);
  m_traj_pnt_cld->header.frame_id = "/map";
#endif
}

UAVLocalMotionPlanner::~UAVLocalMotionPlanner()
{
  if (m_edt_map)
  {
    m_edt_map->free_device();
    delete m_edt_map;
  }

  if(m_hst_map)
    delete m_hst_map;
}

void UAVLocalMotionPlanner::map_call_back(const cpc_aux_mapping::grid_map::ConstPtr &msg)
{
  m_received_map = true;
  if (m_edt_map == nullptr)
  {
    CUDA_GEO::pos origin(msg->x_origin,msg->y_origin,msg->z_origin);
    int3 edt_map_size = make_int3(msg->x_size,msg->y_size,msg->z_size);
    m_edt_map = new EDTMap(origin,msg->width,edt_map_size);
    m_edt_map->setup_device();
    if (m_use_hst_map)
    {
      m_hst_map = new GridGraph(msg->x_size,msg->y_size,msg->z_size);
      m_hst_map->copyEdtData(msg);
    }
  }
  else
  {
    m_edt_map->m_origin = CUDA_GEO::pos(msg->x_origin,msg->y_origin,msg->z_origin);
    m_edt_map->m_grid_step = msg->width;
    if (m_use_hst_map)
    {
      m_hst_map->copyEdtData(msg);
    }
  }
  CUDA_MEMCPY_H2D(m_edt_map->m_sd_map,msg->payload8.data(),static_cast<size_t>(m_edt_map->m_byte_size));
}

void UAVLocalMotionPlanner::vehicle_pose_call_back(const nav_msgs::Odometry::ConstPtr &msg)
{
  m_pose_received = true;
  m_pose = *msg;
}

void UAVLocalMotionPlanner::cycle_process_based_on_status()
{
  switch (m_fly_status) {
  case UAV::AT_GROUND:
  {
    do_at_ground();
    break;
  }
  case UAV::TAKING_OFF:
  {
    do_taking_off();
    break;
  }
  case UAV::IN_AIR:
  {
    do_in_air();
    break;
  }
  case UAV::STUCK:
  {
    do_stuck();
    break;
  }
  case UAV::EMERGENT:
  {
    do_emergent();
    break;
  }
  case UAV::BRAKING:
  {
    do_braking();
    break;
  }
  default:
    break;
  }
}

bool UAVLocalMotionPlanner::is_stuck(const std::vector<UAV::UAVModel::State> &traj, std::vector<JLT::State> yaw_traj, const float3 &target_pos)
{
    if (m_fly_status <= UAV::TAKING_OFF)
        return false;

    bool far_from_tgt = false;
    bool no_turning = false;
    bool no_moving_intention = false;

    // check is it far from target
    if (!is_pos_reached(traj[0],target_pos))
        far_from_tgt = true;

    // check whether the vehicle is about to turn
    float max_turn = 0;
    float turn;
    JLT::State ini_yaw_state = yaw_traj[0];
    for (JLT::State yaw_state : yaw_traj)
    {
      turn = fabsf(yaw_state.p - ini_yaw_state.p);
      if (turn > max_turn)
          max_turn = turn;
    }
    if (max_turn < 0.25f)
        no_turning = true;

    // check whether the vehicle is about to move
    float max_dist = 0;
    float dist;
    UAV::UAVModel::State ini_s = traj[0];
    float3 shift = traj[0].p;
    for (UAV::UAVModel::State s : traj)
    {
        shift = ini_s.p - s.p;
        dist = sqrtf(dot(shift,shift));
        if (dist > max_dist)
            max_dist = dist;
    }
    if (max_dist < 0.4f)
        no_moving_intention = true;

    // update stuck probability
    if (far_from_tgt && no_turning && no_moving_intention)
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

void UAVLocalMotionPlanner::add_to_ref_msg(cpc_motion_planning::ref_data& ref_msg, int ref_counter, const UAV::UAVModel::State &traj_s, const JLT::State &yaw_state)
{
  ref_msg.ids.push_back(ref_counter);
  ref_msg.data.push_back(traj_s.p.x);
  ref_msg.data.push_back(traj_s.p.y);
  ref_msg.data.push_back(traj_s.p.z);

  ref_msg.data.push_back(traj_s.v.x);
  ref_msg.data.push_back(traj_s.v.y);
  ref_msg.data.push_back(traj_s.v.z);

  ref_msg.data.push_back(traj_s.a.x);
  ref_msg.data.push_back(traj_s.a.y);
  ref_msg.data.push_back(traj_s.a.z);

  ref_msg.data.push_back(yaw_state.p);
  ref_msg.data.push_back(yaw_state.v);
  ref_msg.data.push_back(yaw_state.a);
}

#ifdef SHOWPC
void UAVLocalMotionPlanner::plot_trajectory(const std::vector<UAV::UAVModel::State> &traj)
{
  for (UAV::UAVModel::State traj_s : traj)
  {
    pcl::PointXYZ clrP;
    clrP.x = traj_s.p.x;
    clrP.y = traj_s.p.y;
    clrP.z = traj_s.p.z;
    m_traj_pnt_cld->points.push_back(clrP);
  }
  m_traj_pub.publish(m_traj_pnt_cld);
  m_traj_pnt_cld->clear();
}
#endif

